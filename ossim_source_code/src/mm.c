// #ifdef MM_PAGING
/*
 * PAGING based Memory Management
 * Memory management unit mm/mm.c
 */

#include "mm.h"
#include <stdlib.h>
#include <stdio.h>

/*
 * init_pte - Initialize PTE entry
 */
int init_pte(uint32_t *pte, int pre, int fpn, int drt, int swp, int swptyp, int swpoff) {
    if (pre != 0) {
        if (swp == 0) {
            // Non-swap ~ page online
            if (fpn == 0) 
                return -1; // Invalid setting

            // Valid setting with FPN
            *pte = 0;
            SETBIT(*pte, PAGING_PTE_PRESENT_MASK);
            SETVAL(*pte, fpn, PAGING_PTE_FPN_MASK, PAGING_PTE_FPN_LOBIT);
        } else { 
            // Page swapped
            *pte = 0;
            SETBIT(*pte, PAGING_PTE_PRESENT_MASK);
            SETBIT(*pte, PAGING_PTE_SWAPPED_MASK);
            SETVAL(*pte, swptyp, PAGING_PTE_SWPTYP_MASK, PAGING_PTE_SWPTYP_LOBIT);
            SETVAL(*pte, swpoff, PAGING_PTE_SWPOFF_MASK, PAGING_PTE_SWPOFF_LOBIT);
        }
    }
    return 0;
}

/*
 * pte_set_swap - Set PTE entry for swapped page
 * @pte    : target page table entry (PTE)
 * @swptyp : swap type
 * @swpoff : swap offset
 */
int pte_set_swap(uint32_t *pte, int swptyp, int swpoff) {
    *pte = 0;
    SETBIT(*pte, PAGING_PTE_PRESENT_MASK);
    SETBIT(*pte, PAGING_PTE_SWAPPED_MASK);
    SETVAL(*pte, swptyp, PAGING_PTE_SWPTYP_MASK, PAGING_PTE_SWPTYP_LOBIT);
    SETVAL(*pte, swpoff, PAGING_PTE_SWPOFF_MASK, PAGING_PTE_SWPOFF_LOBIT);

    return 0;
}

/*
 * pte_set_fpn - Set PTE entry for on-line page
 * @pte   : target page table entry (PTE)
 * @fpn   : frame page number (FPN)
 */
int pte_set_fpn(uint32_t *pte, int fpn) {
    *pte = 0;
    SETBIT(*pte, PAGING_PTE_PRESENT_MASK);
    CLRBIT(*pte, PAGING_PTE_SWAPPED_MASK);
    SETVAL(*pte, fpn, PAGING_PTE_FPN_MASK, PAGING_PTE_FPN_LOBIT);

    return 0;
}

/*
 * vmap_page_range - map a range of page at aligned address
 */
int vmap_page_range(struct pcb_t *caller, int addr, int pgnum, struct framephy_struct *frames, struct vm_rg_struct *ret_rg) {
    struct framephy_struct *fpit;
    int pgn = PAGING_PGN(addr);

    ret_rg->rg_start = addr;
    ret_rg->rg_end = addr + pgnum * PAGING_PAGESZ;

    // Map range of frames to address space
    for (int pgit = 0; pgit < pgnum; ++pgit) {
        fpit = frames;
        pte_set_fpn(&caller->mm->pgd[pgn + pgit], fpit->fpn);
        frames = frames->fp_next;
        free(fpit);

        // Tracking for later page replacement activities
        enlist_pgn_node(&caller->mm->fifo_pgn, pgn + pgit);
    }

    return 0;
}

/*
 * alloc_pages_range - allocate req_pgnum of frame in RAM
 * @caller    : caller
 * @req_pgnum : request page num
 * @frm_lst   : frame list
 */
int alloc_pages_range(struct pcb_t *caller, int req_pgnum, struct framephy_struct **frm_lst) {
    for (int pgit = 0; pgit < req_pgnum; pgit++) {
        struct framephy_struct *newfp_str = (struct framephy_struct *)malloc(sizeof(struct framephy_struct));
        int fpn;

        if (MEMPHY_get_freefp(caller->mram, &fpn) == 0) {
            newfp_str->fpn = fpn;
        } else {
            int vicpgn, swpfpn;
            if (find_victim_page(caller->mm, &vicpgn) == -1 || MEMPHY_get_freefp(caller->active_mswp, &swpfpn) == -1) {
                if (*frm_lst == NULL) {
                    return -1; // Not enough frames
                } else {
                    // Clean up allocated frames
                    struct framephy_struct *freefp_str;
                    while (*frm_lst != NULL) {
                        freefp_str = *frm_lst;
                        *frm_lst = (*frm_lst)->fp_next;
                        free(freefp_str);
                    }
                    return -3000; // Out of memory
                }
            }

            uint32_t vicpte = caller->mm->pgd[vicpgn];
            int vicfpn = PAGING_FPN(vicpte);
            __swap_cp_page(caller->mram, vicfpn, caller->active_mswp, swpfpn);
            pte_set_swap(&caller->mm->pgd[vicpgn], 0, swpfpn);
            newfp_str->fpn = vicfpn;
        }

        newfp_str->fp_next = *frm_lst;
        *frm_lst = newfp_str;
    }

    return 0;
}

/*
 * vm_map_ram - do the mapping all vm to RAM storage device
 * @caller    : caller
 * @astart    : VM area start
 * @aend      : VM area end
 * @mapstart  : start mapping point
 * @incpgnum  : number of mapped pages
 * @ret_rg    : returned region
 */
int vm_map_ram(struct pcb_t *caller, int astart, int aend, int mapstart, int incpgnum, struct vm_rg_struct *ret_rg) {
    struct framephy_struct *frm_lst = NULL;
    int ret_alloc = alloc_pages_range(caller, incpgnum, &frm_lst);

    if (ret_alloc < 0) {
        if (ret_alloc == -3000) {
            printf("OOM: vm_map_ram out of memory\n");
        }
        return -1; // Out of memory or failed allocation
    }

    // Map the pages
    vmap_page_range(caller, mapstart, incpgnum, frm_lst, ret_rg);

    return 0;
}

/* Swap copy content page from source frame to destination frame */
int __swap_cp_page(struct memphy_struct *mpsrc, int srcfpn, struct memphy_struct *mpdst, int dstfpn) {
    for (int cellidx = 0; cellidx < PAGING_PAGESZ; cellidx++) {
        int addrsrc = srcfpn * PAGING_PAGESZ + cellidx;
        int addrdst = dstfpn * PAGING_PAGESZ + cellidx;

        BYTE data;
        MEMPHY_read(mpsrc, addrsrc, &data);
        MEMPHY_write(mpdst, addrdst, data);
    }
    return 0;
}

/* Initialize an empty Memory Management instance */
int init_mm(struct mm_struct *mm, struct pcb_t *caller) {
    struct vm_area_struct *vma = malloc(sizeof(struct vm_area_struct));

    mm->pgd = malloc(PAGING_MAX_PGN * sizeof(uint32_t));
    vma->vm_id = 0;
    vma->vm_start = 0;
    vma->vm_end = 0;
    vma->sbrk = 0;
    struct vm_rg_struct *first_rg = init_vm_rg(vma->vm_start, vma->vm_end);
    enlist_vm_rg_node(&vma->vm_freerg_list, first_rg);

    vma->vm_next = NULL;
    vma->vm_mm = mm; // Point back to vma owner
    mm->mmap = vma;

    return 0;
}

struct vm_rg_struct *init_vm_rg(int rg_start, int rg_end) {
    struct vm_rg_struct *rgnode = malloc(sizeof(struct vm_rg_struct));

    rgnode->rg_start = rg_start;
    rgnode->rg_end = rg_end;
    rgnode->rg_next = NULL;

    return rgnode;
}

int enlist_vm_rg_node(struct vm_rg_struct **rglist, struct vm_rg_struct *rgnode) {
    rgnode->rg_next = *rglist;
    *rglist = rgnode;

    return 0;
}

int enlist_pgn_node(struct pgn_t **plist, int pgn) {
    struct pgn_t *pnode = malloc(sizeof(struct pgn_t));

    pnode->pgn = pgn;
    pnode->pg_next = *plist;
    *plist = pnode;

    return 0;
}

/*
 * Printing Functions - Only for debugging
 */
int print_list_fp(struct framephy_struct *ifp) {
    struct framephy_struct *fp = ifp;

    if (!fp) {
        printf("NULL list\n");
        return -1;
    }

    while (fp != NULL) {
        printf("fp[%d]\n", fp->fpn);
        fp = fp->fp_next;
    }

    return 0;
}

int print_list_rg(struct vm_rg_struct *irg) {
    if (!irg) {
        printf("NULL list\n");
        return -1;
    }

    while (irg != NULL) {
        printf("rg[%ld->%ld]\n", irg->rg_start, irg->rg_end);
        irg = irg->rg_next;
    }

    return 0;
}

int print_list_vma(struct vm_area_struct *ivma) {
    if (!ivma) {
        printf("NULL list\n");
        return -1;
    }

    while (ivma != NULL) {
        printf("va[%ld->%ld]\n", ivma->vm_start, ivma->vm_end);
        ivma = ivma->vm_next;
    }

    return 0;
}

int print_list_pgn(struct pgn_t *ip) {
    if (!ip) {
        printf("NULL list\n");
        return -1;
    }

    while (ip != NULL) {
        printf("va[%d]-\n", ip->pgn);
        ip = ip->pg_next;
    }

    return 0;
}

int print_pgtbl(struct pcb_t *caller, uint32_t start, uint32_t end) {
    int pgn_start = PAGING_PGN(start);
    int pgn_end = PAGING_PGN(end);

    printf("print_pgtbl: %d - %d\n", start, end);

    if (!caller) {
        printf("NULL caller\n");
        return -1;
    }

    for (int pgit = pgn_start; pgit < pgn_end; pgit++) {
        printf("%08ld: %08x\n", pgit * sizeof(uint32_t), caller->mm->pgd[pgit]);
    }

    for (int pgit = pgn_start; pgit < pgn_end; pgit++) {
        printf("Page Number: %d -> Frame Number: %d\n", pgit, PAGING_FPN(caller->mm->pgd[pgit]));
    }

    printf("================================================================\n");

    return 0;
}

// #endif

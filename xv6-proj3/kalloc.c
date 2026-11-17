// Physical memory allocator, intended to allocate
// memory for user processes, kernel stacks, page table pages,
// and pipe buffers. Allocates 4096-byte pages.

#include "types.h"
#include "defs.h"
#include "param.h"
#include "memlayout.h"
#include "mmu.h"
#include "spinlock.h"

void freerange(void *vstart, void *vend);
extern char end[]; // first address after kernel loaded from ELF file
                   // defined by the kernel linker script in kernel.ld

struct run {
  struct run *next;
};

struct {
  struct spinlock lock;
  int use_lock;
  struct run *freelist;
} kmem;

struct {
  int num_free_pages;
  uint refcount[PHYSTOP >> PGSHIFT];
} pmem;

static uint
page_index(uint pa)
{
  if(pa % PGSIZE || pa < V2P(end) || pa >= PHYSTOP)
    panic("refcount: invalid page");
  return pa >> PGSHIFT;
}

// Initialization happens in two phases.
// 1. main() calls kinit1() while still using entrypgdir to place just
// the pages mapped by entrypgdir on free list.
// 2. main() calls kinit2() with the rest of the physical pages
// after installing a full page table that maps them on all cores.
void
kinit1(void *vstart, void *vend)
{
  initlock(&kmem.lock, "kmem");
  kmem.use_lock = 0;
  memset(pmem.refcount, 0, sizeof(pmem.refcount));
  pmem.num_free_pages = 0;
  freerange(vstart, vend);
}

void
kinit2(void *vstart, void *vend)
{
  freerange(vstart, vend);
  kmem.use_lock = 1;
}

void
freerange(void *vstart, void *vend)
{
  char *p;
  p = (char*)PGROUNDUP((uint)vstart);
  for(; p + PGSIZE <= (char*)vend; p += PGSIZE){
    // Newly discovered pages need a reference before kfree() drops it.
    inc_refcount(V2P(p));
    kfree(p);
  }
}
//PAGEBREAK: 21
// Free the page of physical memory pointed at by v,
// which normally should have been returned by a
// call to kalloc().  (The exception is when
// initializing the allocator; see kinit above.)
void
kfree(char *v)
{
  struct run *r;
  uint index;

  if((uint)v % PGSIZE || v < end || V2P(v) >= PHYSTOP)
    panic("kfree");

  index = page_index(V2P(v));
  if(kmem.use_lock)
    acquire(&kmem.lock);
  if(pmem.refcount[index] == 0)
    panic("kfree: no reference");
  pmem.refcount[index]--;
  if(pmem.refcount[index] != 0){
    if(kmem.use_lock)
      release(&kmem.lock);
    return;
  }

  // Other mappings must be gone before the page can be overwritten.
  memset(v, 1, PGSIZE);
  r = (struct run*)v;
  r->next = kmem.freelist;
  kmem.freelist = r;

  pmem.num_free_pages++;

  if(kmem.use_lock)
    release(&kmem.lock);
}

// Allocate one 4096-byte page of physical memory.
// Returns a pointer that the kernel can use.
// Returns 0 if the memory cannot be allocated.
char*
kalloc(void)
{
  struct run *r;
  uint index;

  if(kmem.use_lock)
    acquire(&kmem.lock);
  r = kmem.freelist;
  if(r){
    index = page_index(V2P(r));
    if(pmem.refcount[index] != 0)
      panic("kalloc: page in use");
    kmem.freelist = r->next;
    pmem.refcount[index] = 1;
    pmem.num_free_pages--;
  }

  if(kmem.use_lock)
    release(&kmem.lock);
  return (char*)r;
}

int
freemem(void)
{
  int count;

  if(kmem.use_lock)
    acquire(&kmem.lock);
  count = pmem.num_free_pages;
  if(kmem.use_lock)
    release(&kmem.lock);
  return count;
}

uint
get_refcount(uint pa)
{
  uint index, count;

  index = page_index(pa);
  if(kmem.use_lock)
    acquire(&kmem.lock);
  count = pmem.refcount[index];
  if(kmem.use_lock)
    release(&kmem.lock);
  return count;
}

void
inc_refcount(uint pa)
{
  uint index;

  index = page_index(pa);
  if(kmem.use_lock)
    acquire(&kmem.lock);
  if(pmem.refcount[index] == (uint)-1)
    panic("refcount: overflow");
  pmem.refcount[index]++;
  if(kmem.use_lock)
    release(&kmem.lock);
}

// Release one reference; kfree() also recycles the page when it reaches zero.
void
dec_refcount(uint pa)
{
  page_index(pa);
  kfree(P2V(pa));
}

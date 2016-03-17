#include <nautilus/nautilus.h>
#include <dev/pci.h>
#include <dev/virtio_pci.h>
#include <dev/virtio_ring.h>
#include <dev/virtio_block.h>
#include <dev/apic.h>
#include <nautilus/irq.h>

#ifndef NAUT_CONFIG_DEBUG_VIRTIO_PCI
#undef DEBUG_PRINT
#define DEBUG_PRINT(fmt, args...)
#endif

// set to 1 to use memory mapped regs
// set to 0 to use ioport mapped regs
// leave at 0 for time being...
#define ACCESS_VIA_MEM 0

#ifndef INFO
#define INFO(fmt, args...) printk("VIRTIO_PCI: " fmt, ##args)
#endif
#ifndef DEBUG
#define DEBUG(fmt, args...) DEBUG_PRINT("VIRTIO_PCI: DEBUG: " fmt, ##args)
#endif
#ifndef ERROR
#define ERROR(fmt, args...) printk("VIRTIO_PCI: ERROR: " fmt, ##args)
#endif

// list of virtio devices we are managing
static struct list_head dev_list;

static struct apic_dev * apic;

// common register offsets
#define DEVICE_FEATURES 0x0    // 4 byte
#define GUEST_FEATURES  0x4    // 4 byte
#define QUEUE_ADDR      0x8    // 4 byte
#define QUEUE_SIZE      0xc    // 2 byte
#define QUEUE_SEL       0xe    // 2 byte
#define QUEUE_NOTIFY    0x10   // 2 byte
#define DEVICE_STATUS   0x12   // 1 byte
#define ISR_STATUS      0x13   // 1 byte
#define CONFIG_VEC      0x14   // 2 byte
#define QUEUE_VEC       0x16   // 2 byte

/* Vector used to disable MSI for queue*/
#define VIRTIO_MSI_NO_VECTOR 0xfff

inline static uint32_t read_regl(struct virtio_pci_dev *dev, uint32_t offset)
{
  uint32_t result;
#if ACCESS_VIA_MEM
  // we want to be assured that we are doing a single read
  // without any compiler nonsense
  uint64_t addr = dev->mem_start + offset;
  //  DEBUG("addr=%p\n",addr);
  __asm__ __volatile__ ("movl (%1),%0"
			: "=r"(result)
			: "r"(addr)
			: "memory");
  return result;
#else
  return inl(dev->ioport_start+offset);
#endif
}


inline static void write_regl(struct virtio_pci_dev *dev, uint32_t offset, uint32_t data)
{
#if ACCESS_VIA_MEM
  // we want to be assured that we are doing a single write
  // without any compiler nonsense
  uint64_t addr = dev->mem_start + offset;
  __asm__ __volatile__ ("movl %1, (%0)"
			: "=r"(addr)
			: "r"(data)
			: "memory");
#else
  outl(data,dev->ioport_start+offset);
#endif
}

uint32_t read_regw(struct virtio_pci_dev *dev, uint32_t offset)
{
  uint16_t result;
#if ACCESS_VIA_MEM
  // we want to be assured that we are doing a single read
  // without any compiler nonsense
  uint64_t addr = dev->mem_start + offset;
  __asm__ __volatile__ ("movw (%1),%0"
			: "=r"(result)
			: "r"(addr)
			: "memory");
  return result;
#else
  return inw(dev->ioport_start+offset);
#endif
}

inline static void write_regw(struct virtio_pci_dev *dev, uint32_t offset, uint16_t data)
{
#if ACCESS_VIA_MEM
  // we want to be assured that we are doing a single write
  // without any compiler nonsense
  uint64_t addr = dev->mem_start + offset;
  __asm__ __volatile__ ("movw %1, (%0)"
			: "=r"(addr)
			: "r"(data)
			: "memory");
#else
  outw(data,dev->ioport_start+offset);
#endif
}

inline static uint32_t read_regb(struct virtio_pci_dev *dev, uint32_t offset)
{
  uint8_t result;
#if ACCESS_VIA_MEM
  // we want to be assured that we are doing a single read
  // without any compiler nonsense
  uint64_t addr = dev->mem_start + offset;
  __asm__ __volatile__ ("movb (%1),%0"
			: "=r"(result)
			: "r"(addr)
			: "memory");
  return result;
#else
  return inb(dev->ioport_start+offset);
#endif
}

inline static void write_regb(struct virtio_pci_dev *dev, uint32_t offset, uint8_t data)
{
#if ACCESS_VIA_MEM
  // we want to be assured that we are doing a single write
  // without any compiler nonsense
  uint64_t addr = dev->mem_start + offset;
  __asm__ __volatile__ ("movb %1, (%0)"
			: "=r"(addr)
			: "r"(data)
			: "memory");
#else
  outb(data,dev->ioport_start+offset);
#endif
}


static int discover_devices(struct pci_info *pci)
{
  struct list_head *curbus, *curdev;
  int num=0;

  DEBUG("Discovering and naming virtio devices\n");

  INIT_LIST_HEAD(&dev_list);

  if (!pci) { 
    ERROR("No PCI info\n");
    return -1;
  }

  list_for_each(curbus,&(pci->bus_list)) { 
    struct pci_bus *bus = list_entry(curbus,struct pci_bus,bus_node);

    DEBUG("Searching PCI bus %u for Virtio devices\n", bus->num);

    list_for_each(curdev, &(bus->dev_list)) { 
      struct pci_dev *pdev = list_entry(curdev,struct pci_dev,dev_node);
      struct pci_cfg_space *cfg = &pdev->cfg;

      DEBUG("Device %u is a %x:%x\n", pdev->num, cfg->vendor_id, cfg->device_id);

      if (cfg->vendor_id==0x1af4 && cfg->device_id>=0x1000 && cfg->device_id<=0x103f) {
	DEBUG("Virtio Device Found (subsys_id=0x%x)\n",cfg->dev_cfg.subsys_id);
	struct virtio_pci_dev *vdev;

	vdev = malloc(sizeof(struct virtio_pci_dev));
	if (!vdev) {
	  ERROR("Cannot allocate device\n");
	  return -1;
	}

	memset(vdev,0,sizeof(*vdev));
	
	vdev->pci_dev = pdev;

	switch (cfg->dev_cfg.subsys_id) { 
	case 0x1:
	  DEBUG("Net Device\n");
	  vdev->type = VIRTIO_PCI_NET;
	  break;
	case 0x2:
	  DEBUG("Block Device\n");
	  vdev->type = VIRTIO_PCI_BLOCK;
	  break;
	default:
	  DEBUG("Other Device\n");
	  vdev->type = VIRTIO_PCI_OTHER;
	  break;
	}

	snprintf(vdev->name,32, "virtio-%d-%s", num, 
		 vdev->type==VIRTIO_PCI_NET ? "net" :
		 vdev->type==VIRTIO_PCI_BLOCK ? "block" :
		 vdev->type==VIRTIO_PCI_OTHER ? "other" : "UNKNOWN");


	// PCI Interrupt (A..D)
	vdev->pci_intr = cfg->dev_cfg.intr_pin;
	// Figure out mapping here or look at capabilities for MSI-X
	// vdev->intr_vec = ...

	// we expect two bars exist, one for memory, one for i/o
	// and these will be bar 0 and 1
	// check to see if there are no others
	for (int i=0;i<6;i++) { 
	  uint32_t bar = pci_cfg_readl(bus->num,pdev->num, 0, 0x10 + i*4);
	  uint32_t size;
	  DEBUG("bar %d: 0x%0x\n",i, bar);
	  if (i>=2 && bar!=0) { 
	    DEBUG("Not expecting this to be a non-empty bar...\n");
	  }
	  if (!(bar & 0x1)) { 
	    // handle only 32 bit memory for now
	    uint8_t mem_bar_type = (bar & 0x6) >> 1;
	    if (mem_bar_type != 0) { 
	      ERROR("Cannot handle memory bar type 0x%x\n", mem_bar_type);
	      return -1;
	    }
	  }

	  // determine size
	  // write all 1s, get back the size mask
	  pci_cfg_writel(bus->num,pdev->num,0,0x10 + i*4, 0xffffffff);
	  // size mask comes back + info bits
	  size = pci_cfg_readl(bus->num,pdev->num,0,0x10 + i*4);

	  // mask all but size mask
	  if (bar & 0x1) { 
	    // I/O
	    size &= 0xfffffffc;
	  } else {
	    // memory
	    size &= 0xfffffff0;
	  }
	  size = ~size;
	  size++; 

	  // now we have to put back the original bar
	  pci_cfg_writel(bus->num,pdev->num,0,0x10 + i*4, bar);

	  if (!size) { 
	    // non-existent bar, skip to next one
	    continue;
	  }

	  if (size>0 && i>=2) { 
	    ERROR("unexpected virtio pci bar with size>0!\n");
	    return -1;
	  }
	  
	  if (bar & 0x1) { 
	    vdev->ioport_start = bar & 0xffffffc0;
	    vdev->ioport_end = vdev->ioport_start + size;
	  } else {
	    vdev->mem_start = bar & 0xfffffff0;
	    vdev->mem_end = vdev->mem_start + size;
	  }

	}

	// Now we need to figure out its interrupt
	if (pci_cfg_has_capability(bus->num,pdev->num,0,PCI_CAP_ID_MSIX)) { 
	  DEBUG("device supports MSI-X\n");
	} else {
	  DEBUG("device does not support MSI-X\n");
	}

	INFO("Adding virtio %s device with name %s : bus=%u dev=%u func=%u: pci_intr=%u intr_vec=%u ioport_start=%p ioport_end=%p mem_start=%p mem_end=%p\n",
	     vdev->type==VIRTIO_PCI_BLOCK ? "block" :
	     vdev->type==VIRTIO_PCI_NET ? "net" : "other",
	     vdev->name,
	     bus->num, pdev->num, 0,
	     vdev->pci_intr, vdev->intr_vec,
	     vdev->ioport_start, vdev->ioport_end,
	     vdev->mem_start, vdev->mem_end);
	     

	list_add_tail(&vdev->virtio_node,&dev_list);
	num++;

      }
    }
  }
  return 0;
}


#define ALIGN(x) (((x) + 4095UL) & ~4095UL) 

#define NUM_PAGES(x) ((x)/4096 + !!((x)%4096))


static inline unsigned compute_size(unsigned int qsz) 
{ 
     return ALIGN(sizeof(struct virtq_desc)*qsz + sizeof(uint16_t)*(3 + qsz)) 
          + ALIGN(sizeof(uint16_t)*3 + sizeof(struct virtq_used_elem)*qsz); 
}

int virtio_ring_init(struct virtio_pci_dev *dev)
{
  uint16_t i;
  uint64_t qsz;
  uint64_t qsz_numbytes;
  uint64_t alloc_size;
  

  DEBUG("Ring init of %s\n",dev->name);

  // now let's figure out the ring sizes
  dev->num_vrings=0;
  for (i=0;i<MAX_VRINGS;i++) {
    
    // select the queue we are interested in
    write_regw(dev,QUEUE_SEL,i);
    qsz = read_regw(dev,QUEUE_SIZE);
    if (qsz==0) {
      // out of queues to support
      break;
    }
    INFO("Ring %u has 0x%lx slots\n", i, qsz);
    qsz_numbytes = compute_size(qsz);
    INFO("Ring %u has size 0x%lx bytes\n", i, qsz_numbytes);


    dev->vring[i].size_bytes = qsz_numbytes;
    alloc_size = 4096 * (NUM_PAGES(qsz_numbytes) + 1);

    if (!(dev->vring[i].data = malloc(alloc_size))) {
      ERROR("Cannot allocate ring\n");
      return -1;
    }

    memset(dev->vring[i].data,0,alloc_size);

    dev->vring[i].aligned_data = (uint8_t *)ALIGN((uint64_t)(dev->vring[i].data));
    
    dev->vring[i].vq.num = qsz;

    dev->vring[i].vq.desc = (struct virtq_desc *) (dev->vring[i].aligned_data);

    dev->vring[i].vq.avail = (struct virtq_avail *) 
      (dev->vring[i].aligned_data 
       + sizeof(struct virtq_desc)*qsz);

    dev->vring[i].vq.used = (struct virtq_used *) 
      (dev->vring[i].aligned_data
       +  ALIGN(sizeof(struct virtq_desc)*qsz + sizeof(uint16_t)*(3 + qsz))); 


    DEBUG("ring allocation at %p for 0x%lx bytes\n", dev->vring[i].data,alloc_size);
    DEBUG("ring data at %p\n", dev->vring[i].aligned_data);
    DEBUG("ring num  = 0x%lx\n",dev->vring[i].vq.num);
    DEBUG("ring desc at %p\n", dev->vring[i].vq.desc);
    DEBUG("ring avail at %p\n", dev->vring[i].vq.avail);
    DEBUG("ring used at %p\n", dev->vring[i].vq.used);

    // now tell device about the ring
    // note it's a 32 bit register, but the address is a page address
    // so it really represents a 44 bit address (32 bits * 4096)
    write_regl(dev,QUEUE_ADDR,(uint32_t)(((uint64_t)(dev->vring[i].aligned_data))/4096));
	
    
    //virtq_init(&dev->vring[i].vq, size, dev->vring[i].aligned_data, 4096);
    //INFO("Initialized Virtio vring with %d elements\n",size);

/*    for (uint16_t j=0;j<qsz-1;j++){
       dev->vring[i].vq.desc[j].next = j + 1;       
    }*/

    dev->num_vrings++;
  }

  if (i==MAX_VRINGS) { 
    ERROR("Device needs to many rings\n");
    return -1;
  }
    
  return 0;
}

// a descriptor with len 0 will denote
// it is free
// we return -1 if the allocation cannot occur
static uint32_t allocate_descriptor(volatile struct virtq *vq)
{
  uint32_t i;

  // this is hideous
  for (i=0;i<vq->num;i++) { 
    if (!vq->desc[i].len) { 
      // set to nonzero sentinal value
      vq->desc[i].len=0xdeadbeef;
      DEBUG("Allocate descriptor %u\n",i);
      return i;
    }
  }
  return -1;
}

static void read_request_process(volatile struct virtq *vq, uint32_t i, uint32_t head_id, uint32_t *total)
{ 
  unsigned char *char_addr = (unsigned char*)(vq->desc[i].addr);
  INFO("Data is at: %x\n", char_addr);
  INFO("Data is: %s\n", char_addr);
  
  if (i == head_id){
    (*total)++;
    read_request_process(vq, vq->desc[i].next, head_id, total);
  }
  else if(vq->desc[i].next == 0){
    (*total)++;
    return;
  }
  else{
//    struct virti = (struct virtio_block_request*)vq->desc[i].addr;
    //for (int i=0;i<sizeof(return_blkrq->data);i++){
    /*unsigned char *char_addr = (uint64_t)(vq->desc[i].addr);
      INFO("Data is at: %x\n", (uint64_t)(vq->desc[i].addr));
      INFO("Data is: %c\n", char_addr[0]);*/
    //}
    (*total)++;
    //nk_dump_mem(char_addr, 512);
    read_request_process(vq, vq->desc[i].next, head_id, total);
  }
}

// doesn't do anything except counting the numbers of descriptors to free
static void write_flush_request_process(volatile struct virtq *vq, uint32_t i, uint32_t head_id, uint32_t *total)
{ 
  if (i == head_id){
    (*total)++;
    write_flush_request_process(vq, vq->desc[i].next, head_id, total);
  }
    /*struct virtio_block_request* return_blkrq = (struct virtio_block_request*)vq->desc[i].addr;
      INFO("Data is: %s\n", return_blkrq->data);*/
  else if(vq->desc[i].next == 0){
    (*total)++;
    return;
  }
  else{
    (*total)++;
    write_flush_request_process(vq, vq->desc[i].next, head_id, total);
  }
}

static int check_status(volatile struct virtq *vq, uint32_t i)
{ 
  if (vq->desc[i].next == 0){
    //struct virtio_block_request* return_blkrq = (struct virtio_block_request*)vq->desc[i].addr;
    uint8_t return_status = *(uint8_t *)(vq->desc[i].addr);
    //uint8_t return_status = return_blkrq->status;
//    uint8_t return_status = *return_blkrq;
    DEBUG("Status bit of last request packet is %d\n", return_status);
    switch(return_status){
      case VIRTIO_BLK_S_OK:
        DEBUG("Block request OK\n");
        break;
      case VIRTIO_BLK_S_IOERR:
        DEBUG("Block request encountered device or driver error\n");
        return -1;
      case VIRTIO_BLK_S_UNSUPP:
        DEBUG("Block request unsupported by device\n");
        return -1;
      default:
        DEBUG("Unknown status returned by device\n");
        return -1;
    }
    return 0;
  }
  else{
    struct virtio_block_request* return_blkrq = (struct virtio_block_request*)vq->desc[i].addr;
    uint8_t return_status = return_blkrq->status;
    return check_status(vq, vq->desc[i].next);
  }
}

static int free_descriptor(volatile struct virtq *vq, uint32_t i)
{
  DEBUG("Freeing descriptor %u\n",i);
  if (!vq->desc[i].len) { 
    DEBUG("Warning: descriptor already appears freed\n");
    return 0; 
  }
  
  if (vq->desc[i].next == 0){
    vq->desc[i].len = 0; 
    return 0;
  }
  else{
    vq->desc[i].len = 0; 
    return free_descriptor(vq, vq->desc[i].next);
  }
}

static int process_descriptor(volatile struct virtq *vq, uint32_t head_id, uint32_t *total)
{
  int blkrq_status = check_status(vq, head_id);
  /*if (blkrq_status){
    DEBUG("bad block request result status, now freeing descriptors\n");
    write_flush_request_process(vq, head_id, head_id, total); 
    return free_descriptor(vq, head_id, total);
  }*/

  struct virtio_block_request* return_blkrq = (struct virtio_block_request*)vq->desc[head_id].addr;
  DEBUG("Buffer located at 0x%x\n", return_blkrq);
  uint32_t return_type = return_blkrq->type;
  switch(return_type){
    case 0:
      DEBUG("Processing read request\n");
      read_request_process(vq, head_id, head_id, total);
      break;
    case 1: 
      DEBUG("Processing finished write request\n");
      write_flush_request_process(vq, head_id, head_id, total);
      break;
    default:
      DEBUG("Processing finished flush request\n");
      write_flush_request_process(vq, head_id, head_id, total);
  }
  DEBUG("Freeing descriptors\n");
  return free_descriptor(vq, head_id);
}

// Returns 0 if we are able to place the request
// into a descriptor and queue it to the avail ring
// returns nonzero if this fails
int virtio_enque_request(struct virtio_pci_dev *dev,
				uint32_t ring, 
				uint64_t addr, 
				uint32_t len, 
				uint16_t flags,
                                uint8_t head,
                                uint8_t tail)
{
  volatile struct virtq *vq = &dev->vring[ring].vq;

  uint32_t i;
  
  i = allocate_descriptor(vq);
  DEBUG("allocated descriptor %x\n", i);
  if (i==-1) { 
    return -1;
  }
  
  vq->desc[i].addr=addr;
  vq->desc[i].len=len;
  vq->desc[i].flags=flags;
  //if (flags != 2 && flags!=0) vq->desc[i].next=i+1;
  if (tail != 1) vq->desc[i].next=i+1;
 
  DEBUG("vq->desc[%d] = (addr %p, len %x, flags %x,next %x)\n", i, vq->desc[i].addr, vq->desc[i].len, vq->desc[i].flags, vq->desc[i].next);
  DEBUG("before: vq->avail->idx = %x mod at %x , vq->num= %x\n", vq->avail->idx, vq->avail->idx % vq->num, vq->num);

  //vq->avail[i].flags = 0;
  vq->avail->flags = 0;

  if (head == 1) vq->avail->ring[vq->avail->idx % vq->num] = i; // we only put the index of the HEAD of the desc chain in avail.ring[]
  __asm__ __volatile__ ("" : : : "memory"); // software memory barrier
  __sync_synchronize(); // hardware memory barrier
  vq->avail->idx++; // it is ok that this wraps around
  __asm__ __volatile__ ("" : : : "memory"); // software memory barrier
  __sync_synchronize(); // hardware memory barrier

   
  DEBUG("after: vq->avail->idx = %x mod at %x , vq->num= %x\n", vq->avail->idx, vq->avail->idx % vq->num, vq->num);
  
  return 0;
}

// Processing outstanding responses
int virtio_dequeue_responses(struct virtio_pci_dev *dev,
				    uint32_t ring)
{
  struct virtio_pci_vring *vring = &dev->vring[ring];
  volatile struct virtq *vq = &vring->vq;
  uint16_t avail_flags;
  
  avail_flags = vq->avail->flags;

  // disable interrupts
  vq->avail->flags |= VIRTQ_AVAIL_F_NO_INTERRUPT;

  while (1) { 

    
    for (int i = 0;i<20;i++){DEBUG("used[%d] = %d with length %x\n", i, vq->used->ring[i].id, vq->used->ring[i].len);}

    struct virtq_used_elem *e = &(vq->used->ring[(vring->last_seen_used) % vq->num]);
    DEBUG("last seen used = %d\n", vring->last_seen_used);
    DEBUG("used idx = %d\n", vq->used->idx);
    
    if (vring->last_seen_used >= vq->used->idx ) {
      //vq->avail->flags = avail_flags;

      __asm__ __volatile__ ("" : : : "memory"); // sw mem barrier
      __sync_synchronize(); // hw mem barrier

      // restore interrupt state to whatever it was previously
      vq->avail->flags = avail_flags;

      // check again
      if (vring->last_seen_used >= vq->used->idx) {
        //DEBUG("here\n");
	break;
      }
      vq->avail->flags |= VIRTQ_AVAIL_F_NO_INTERRUPT;
    } 
    

    /*if (e->len!=1) { 
      DEBUG("Surprising len %x response\n", e->len);
    }*/

    uint32_t total = 0;
    int process_result;
    DEBUG("processing from desc[%d] at used[%d] with length %x\n", e->id, vring->last_seen_used, e->len);
    process_result = process_descriptor(vq, e->id, &total);

    vring->last_seen_used += total;
    //DEBUG("at end: last seen used = %d\n", vring->last_seen_used);
  }

  return 0;
}

// free memory space of vrings
int virtio_ring_deinit(struct virtio_pci_dev *dev)
{
  uint8_t num_vrings = dev->num_vrings;
  uint8_t i;

  for (i=0;i<num_vrings;i++){ 
    write_regw(dev,QUEUE_SEL,i);
    
    // select and deactivate the queue
    write_regl(dev,QUEUE_ADDR,0);

    free(dev->vring[i].aligned_data);
    INFO("Deallocated Virtio vring on device %s\n", dev->name);
    dev->num_vrings--;
  }
  
  return 0;
}

static int virtio_block_handler()
{
  DEBUG("Entering block interrrupt handler\n");
  struct list_head *curdev;
  struct virtio_pci_dev *dev;

  // we assume our virtio-block device is the only one making the interrupt
  list_for_each(curdev,&(dev_list)) { 
    dev = list_entry(curdev,struct virtio_pci_dev,virtio_node);
  }

  // read ISR status to clear
  uint32_t isr_status = read_regb(dev, ISR_STATUS);
  DEBUG("Current ISR status is: %x\n", isr_status);
  isr_status = read_regb(dev, ISR_STATUS);
  DEBUG("Current ISR status is: %x\n", isr_status);
  blockrq_dequeue(dev);
  DEBUG("Leaving block interrrupt handler\n");
  return 0;
}

static int virtio_block_init(struct virtio_pci_dev *dev)
{

  uint32_t val;
  
  volatile struct virtq *vq = &dev->vring[0].vq;

  DEBUG("Block init of %s\n",dev->name);

 /* write_regb(dev,DEVICE_STATUS,0x0); // driver resets device
  write_regb(dev,DEVICE_STATUS,0x01); // driver acknowledges device
  write_regb(dev,DEVICE_STATUS,0x03); // driver can drive device*/

  /*val = read_regl(dev,DEVICE_FEATURES);
  DEBUG("device features: 0x%0x\n",val);
  write_regl(dev,GUEST_FEATURES, 0x100);
  val = read_regl(dev,GUEST_FEATURES);
  DEBUG("guest features set to: 0x%0x\n", val);

  write_regb(dev,DEVICE_STATUS, 0b1011);*/
  
  register_int_handler(228, virtio_block_handler, NULL);
  
  write_regb(dev,DEVICE_STATUS, 0b1111); // driver is now active
  
  struct virtio_block_request writerq[3];
  memset(&writerq, 0, sizeof(writerq));
  writerq[0].type = 1;
  writerq[0].priority = 0;
  writerq[0].sector = 0; 
  memset(&writerq[1].data, 0, 512);
  //memset(&writerq[2].data, 0xff, 512);
  writerq[1].data[0] = 'r'; 
  writerq[1].data[1] = 'a'; 
  writerq[1].data[2] = 'n'; 
  writerq[1].data[3] = 'd'; 
  writerq[1].data[4] = 'o'; 
  writerq[1].data[5] = 'm'; 
  /*writerq[2].data[0] = 'd'; 
  writerq[2].data[1] = 'a'; 
  writerq[2].data[2] = 't'; 
  writerq[2].data[3] = 'a';  */
  writerq[2].status = 5;

  DEBUG("start idx field of used is now %d\n", vq->used->idx);
  int enqueue_result = blockrq_enqueue(dev, writerq, sizeof(writerq)/sizeof(struct virtio_block_request));
  if (enqueue_result == -1){
    ERROR("block request enqueue error\n");
  }
  DEBUG("after enque idx field of used is %d\n", vq->used->idx);
  write_regw(dev,QUEUE_NOTIFY, 0);
  udelay(1000000);
  DEBUG("after notifying the device, idx field of used is %d\n", vq->used->idx);

  struct virtio_block_request flushrq[3];
  memset(&flushrq, 0, sizeof(flushrq));
  flushrq[0].type = 4;
  flushrq[0].priority = 0;
  flushrq[0].sector = 0;
  flushrq[2].status = 5;
 
 /* DEBUG("start idx field of used is now %d\n", vq->used->idx);
  enqueue_result = blockrq_enqueue(dev, flushrq, sizeof(flushrq)/sizeof(struct virtio_block_request));
  if (enqueue_result == -1){
    ERROR("block request enqueue error\n");
  }
  DEBUG("after enque idx field of used is %d\n", vq->used->idx);
  write_regw(dev,QUEUE_NOTIFY, 0);
  udelay(1000000);
  DEBUG("after notifying the device, idx field of used is %d\n", vq->used->idx);*/
  
  struct virtio_block_request readrq[3];
  memset(&readrq, 0, sizeof(readrq));
  memset(&(readrq[2].status), 0, sizeof(readrq[2].status));
  readrq[0].type = 0;
  readrq[0].priority = 0;
  readrq[0].sector = 0;
  //readrq[3].status = 5;
  //memset(&readrq[1].data, 'a', sizeof(readrq[1].data));
  //memset(&readrq[2].data, 'b', sizeof(readrq[2].data));
  //readrq[2].status = 5;
/*  readrq[0].status = 5;
  readrq[1].status = 5;
  readrq[2].status = 5;*/
  //readrq[3].status = 5;

  DEBUG("start idx field of used is now %d\n", vq->used->idx);
  enqueue_result = blockrq_enqueue(dev, readrq, sizeof(readrq)/sizeof(struct virtio_block_request));
  if (enqueue_result == -1){
    ERROR("block request enqueue error\n");
  }
  DEBUG("after enque idx field of used is %d\n", vq->used->idx);
  write_regw(dev,QUEUE_NOTIFY, 0);
  udelay(1000000);
  DEBUG("after notifying the device, idx field of used is %d\n", vq->used->idx);
   
  /*write_regw(dev,QUEUE_VEC, 0);
  if( read_regw(dev, QUEUE_VEC) == VIRTIO_MSI_NO_VECTOR){
    DEBUG("Writing Queue Vector failed");
  }*/

  DEBUG("end of block init\n");
  
  return 0;
}

static int virtio_net_init(struct virtio_pci_dev *dev)
{
  uint32_t val;

  DEBUG("Net init of %s\n",dev->name);

  write_regb(dev,DEVICE_STATUS,0x0); // driver resets device
  write_regb(dev,DEVICE_STATUS,0b1); // driver acknowledges device
  write_regb(dev,DEVICE_STATUS,0b11); // driver can drive device
 
  val = read_regl(dev,DEVICE_FEATURES);
  DEBUG("device features: 0x%0x\n",val);

  return 0;
}

static int bringup_device(struct virtio_pci_dev *dev)
{
  DEBUG("Bringing up %s\n",dev->name);
  uint32_t val;
  switch (dev->type) {
  case VIRTIO_PCI_BLOCK:
    write_regb(dev,DEVICE_STATUS,0x0); // driver resets device
    write_regb(dev,DEVICE_STATUS,0b1); // driver acknowledges device
    write_regb(dev,DEVICE_STATUS,0b11); // driver can drive device
 
    val = read_regl(dev,DEVICE_FEATURES);
    DEBUG("device features: 0x%0x\n",val);
    write_regl(dev,GUEST_FEATURES, 0x00000ed4);
    val = read_regl(dev,GUEST_FEATURES);
    DEBUG("guest features set to: 0x%0x\n", val);

    write_regb(dev,DEVICE_STATUS, 0b1011); //set the FEATURE_OK bit
   
     if (virtio_ring_init(dev)) { 
      ERROR("Failed to bring up device %s\n", dev->name);
      return -1;
    }
    return virtio_block_init(dev);
    break;
  case VIRTIO_PCI_NET:
    if (virtio_ring_init(dev)) { 
      ERROR("Failed to bring up device %s\n", dev->name);
      return -1;
    }
    return virtio_net_init(dev);
    break;
  case VIRTIO_PCI_OTHER:
  default:
    INFO("Skipping unsupported device type\n");
    return 0;
  }
    
}

static int bringup_devices()
{
  struct list_head *curdev;

  DEBUG("Bringing up virtio devices\n");

  list_for_each(curdev,&(dev_list)) { 
    struct virtio_pci_dev *dev = list_entry(curdev,struct virtio_pci_dev,virtio_node);
    if (bringup_device(dev)) { 
      ERROR("Bringup of virtio devices failed\n");
      return -1;
    }
  }

  return 0;
}

int virtio_pci_init(struct naut_info * naut)
{

  INFO("init\n");

  if (discover_devices(naut->sys.pci)) { 
    ERROR("Discovery failed\n");
    return -1;
  }


  bringup_devices();
  
  return 0;
}

int virtio_pci_deinit()
{
  INFO("deinited\n");
  return 0;
}




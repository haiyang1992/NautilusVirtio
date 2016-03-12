#include <nautilus/nautilus.h>
#include <dev/virtio_block.h>

int write_block(void *state, uint64_t blocknum, uint8_t *src){
  // TODO
  struct virtio_block_request *write_rq;
 // write_rq = malloc(sizeof(virtio_block_request) * () + 1);
  write_rq = malloc(sizeof(struct virtio_block_request));
  write_rq[0].type = VIRTIO_BLK_T_OUT;
  write_rq[0].priority = 1;
  //write_rq[0].blocknum = f(blocknum);
  // fit data from src into write_rq[1].data;
  // add final rq for status
  
  //blockrq_enqueue();
  return 0;
}

int check_blkrq_status(struct virtio_pci_dev *dev,				  
		       uint32_t ring,
	               uint64_t addr,
		       uint32_t len,
		       uint16_t flags){
//  struct virtio_pci_vring *vring = dev->vring[ring];
  volatile struct virtq *vq = &(dev->vring[ring].vq);
  DEBUG("next field of desc is %d\n", vq->desc->next);
  if (vq->desc->next == 0){
    uint8_t return_status = vq->desc->addr;
    DEBUG("Status bit of last request packet is %d\n", return_status);
    switch(return_status){
      case VIRTIO_BLK_S_OK:
        DEBUG("Block request OK\n");
        break;
      case VIRTIO_BLK_S_IOERR:
        DEBUG("Block request encountered device or driver error\n");
        break;
      case VIRTIO_BLK_S_UNSUPP:
        DEBUG("Block request unsupported by device\n");
        break;
      default:
        DEBUG("Unknown status returned by device\n");
        return -1;
    }
    DEBUG("addr of desc is 0x%x\n", return_status);
  }
  return 0;
}

int read_block(void *state, uint64_t blocknum, uint8_t *dest){
  // TODO
  struct virtio_block_request read_rq[2];
  read_rq[0].type = VIRTIO_BLK_T_IN;
  read_rq[0].priority = 1;
  //read_rq[0].sector = ...;
  //blockrq_enqueue();
  return 0;
}

int blockrq_enqueue(struct virtio_pci_dev *dev, struct virtio_block_request blkrq[], uint8_t size){  
  int enqueue_status;
  uint16_t i;

  for (i=0;i<size;i++){
    uint8_t head = 0;
    uint8_t tail = 0;
    uint16_t flags = 1;
    if (blkrq[0].type == 0) flags = 2;
    if (i == 0){ 
      head = 1;
      flags = 1;
    }
    if (i == size - 1){
      flags = 2;
      tail = 1;
    }
    enqueue_status = virtio_enque_request(dev, 0, (uint64_t)&blkrq[i], (uint32_t)(sizeof(blkrq[i])), flags, head, tail);
    if (enqueue_status){
      ERROR("Enqueue of block number %d failed\n", i + 1);
      return -1;
    }
  }
  DEBUG("Enqueued block request\n");
  return 0;
}

int blockrq_dequeue(struct virtio_pci_dev *dev){
  //struct virtio_block_request blkrq;
  //memset(&blkrq, 0, sizeof(blkrq));

  uint32_t ring = 0;
  //uint32_t len = sizeof(blkrq);
  //uint16_t flags = 2; 
  int (*call_back)(struct virtio_pci_dev*, uint32_t, uint64_t, uint32_t, uint16_t);
  call_back = &check_blkrq_status;
 
  volatile struct virtq *vq = &(dev->vring[ring].vq);
  DEBUG("About to call dequeue response function\n"); 
  if(virtio_dequeue_responses(dev, ring, call_back)){
     DEBUG("Dequeue response error\n");
     return -1;
  } 
  return 0;
}

/*struct block_dev * block_dev_register(char *name, struct block_dev_int *interface, void *state){
  // TODO
}*/

int block_dev_unregister(struct block_dev *b){
  // TODO
  return 0;
}

#include <nautilus/nautilus.h>
#include <dev/virtio_block.h>

int write_block(void *state, uint64_t blocknum, uint8_t *src){
  // TODO
  struct virtio_block_request *write_rq;
 // write_rq = malloc(sizeof(virtio_block_request) * () + 1);
  write_rq = malloc(sizeof(struct virtio_block_request));
  write_rq[0].type = VIRTIO_BLK_T_OUT;
  write_rq[0].priority = 0;
  //write_rq[0].blocknum = f(blocknum);
  // fit data from src into write_rq[1].data;
  // add final rq for status
  
  //blockrq_enqueue();
  return 0;
}

int read_block(void *state, uint64_t blocknum, uint8_t *dest){
  // TODO
  struct virtio_block_request read_rq[2];
  read_rq[0].type = VIRTIO_BLK_T_IN;
  read_rq[0].priority = 0;
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
      enqueue_status = virtio_enque_request(dev, 0, (uint64_t)&(blkrq[i].type), 128, flags, head, tail);
      //nk_dump_mem((uint64_t)(&blkrq[i].type), 128);
    }
    else if (i == size - 1){
      flags = 2;
      tail = 1;
      enqueue_status = virtio_enque_request(dev, 0, (uint64_t)&(blkrq[i].status), 8, flags, head, tail);
      //nk_dump_mem( (uint64_t)&(blkrq[i].status), 8);
    }
    else{
      enqueue_status = virtio_enque_request(dev, 0, (uint64_t)&(blkrq[i].data), 512, flags, head, tail);
      //nk_dump_mem( (uint64_t)&(blkrq[i].data), 512);
    }
    
    if (enqueue_status){
      ERROR("Enqueue of block number %d failed\n", i + 1);
      return -1;
    }
  }
  DEBUG("Enqueued block request\n");
  return 0;
}

int blockrq_dequeue(struct virtio_pci_dev *dev){

  uint32_t ring = 0;
 
  volatile struct virtq *vq = &(dev->vring[ring].vq);
  DEBUG("About to call dequeue response function\n"); 
  if(virtio_dequeue_responses(dev, ring)){
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

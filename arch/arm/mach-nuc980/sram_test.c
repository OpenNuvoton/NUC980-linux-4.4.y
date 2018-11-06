
#include <linux/module.h>
#include <mach/sram.h>

static int __init MySramTest_init(void) {

	dma_addr_t dma[5];
	void * vaddr[5];

	vaddr[0]=sram_alloc(1024,&dma[0]);
	printk("1.vaddr[0]=0x%08x,dma[0]=0x%08x\n",(u32)vaddr[0],dma[0]);
	vaddr[1]=sram_alloc(1023,&dma[1]);
	printk("2.vaddr[1]=0x%08x,dma[1]=0x%08x\n",(u32)vaddr[1],dma[1]);
	sram_free(vaddr[0],1024);
	printk("3.vaddr[0]=0x%08x,dma[0]=0x%08x\n",(u32)vaddr[0],dma[0]);
	vaddr[2]=sram_alloc(2048,&dma[2]);
	printk("4.vaddr[2]=0x%08x,dma[2]=0x%08x\n",(u32)vaddr[2],dma[2]);
	vaddr[3]=sram_alloc(1024,&dma[3]);
	printk("5.vaddr[3]=0x%08x,dma[3]=0x%08x\n",(u32)vaddr[3],dma[3]);
	vaddr[4]=sram_alloc(1024*32,&dma[4]);
	printk("6.vaddr[4]=0x%08x,dma[4]=0x%08x\n",(u32)vaddr[4],dma[4]);

	return 0;
}

static void __exit MySramTest_exit(void) {
    return;
}

module_init(MySramTest_init);
module_exit(MySramTest_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mister X");

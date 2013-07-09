#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/vmalloc.h>
#include <linux/pagemap.h>
#include <linux/netdevice.h>

#include "igb.h"
#include "igb_vmdq.h"

#if defined(DEBUG) || defined (DEBUG_DUMP) || defined (DEBUG_ICR) || defined(DEBUG_ITR)
#define DRV_DEBUG "_debug"
#else
#define DRV_DEBUG
#endif
#define DRV_HW_PERF
#define VERSION_SUFFIX

#define MAJ 4
#define MIN 1
#define BUILD 2
#define DRV_VERSION __stringify(MAJ) "." __stringify(MIN) "." __stringify(BUILD) VERSION_SUFFIX DRV_DEBUG DRV_HW_PERF

char igb_driver_name[] = "igb-diag";
char igb_driver_version[] = DRV_VERSION;
static const char igb_driver_string[] =
                                "Intel(R) Gigabit Ethernet Network Driver";
static const char igb_copyright[] =
				"Copyright (c) 2007-2012 Intel Corporation.";


static int diag_mode = 0;
module_param(diag_mode, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(diag_mode, "diag_mode integer");

static DEFINE_PCI_DEVICE_TABLE(igb_pci_tbl_diag) = {
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_I211_DIAG) },
	/* required last entry */
	{0, }
};
MODULE_DEVICE_TABLE(pci, igb_pci_tbl_diag);

static int igb_probe_diag(struct pci_dev *, const struct pci_device_id *);
static void __devexit igb_remove_diag(struct pci_dev *pdev);

static int __devinit igb_probe_diag(struct pci_dev *pdev,
			       const struct pci_device_id *ent)
{
	struct net_device *netdev;
	struct igb_adapter *adapter;
	struct e1000_hw *hw;
	int err, pci_using_dac;

	printk("%s > \n",__func__);

	err = pci_enable_device_mem(pdev);
	if (err)
		return err;

	pci_using_dac = 0;
	err = dma_set_mask(pci_dev_to_dev(pdev), DMA_BIT_MASK(64));
	if (!err) {
		err = dma_set_coherent_mask(pci_dev_to_dev(pdev), DMA_BIT_MASK(64));
		if (!err)
			pci_using_dac = 1;
	} else {
		err = dma_set_mask(pci_dev_to_dev(pdev), DMA_BIT_MASK(32));
		if (err) {
			err = dma_set_coherent_mask(pci_dev_to_dev(pdev), DMA_BIT_MASK(32));
			if (err) {
				IGB_ERR("No usable DMA configuration, "
				        "aborting\n");
				goto err_dma;
			}
		}
	}

	pci_set_master(pdev);

	netdev = alloc_etherdev(sizeof(struct igb_adapter));
	SET_NETDEV_DEV(netdev, &pdev->dev);
	pci_set_drvdata(pdev, netdev);
	adapter = netdev_priv(netdev);
	adapter->netdev = netdev;
	adapter->pdev = pdev;
	hw = &adapter->hw;
	hw->back = adapter;

	err = pci_request_selected_regions(pdev,
	                                   pci_select_bars(pdev,
                                                           IORESOURCE_MEM),
	                                   igb_driver_name);
	if (err)
		goto err_pci_reg;

	err = -EIO;
	hw->hw_addr = ioremap(pci_resource_start(pdev, 0),
	                      pci_resource_len(pdev, 0));
	if (!hw->hw_addr)
		goto err_ioremap;

	return 0;

err_ioremap:
	pci_release_selected_regions(pdev,
	                             pci_select_bars(pdev, IORESOURCE_MEM));
err_pci_reg:
err_dma:
	pci_disable_device(pdev);
	return err;
}

static void __devexit igb_remove_diag(struct pci_dev *pdev)
{
	struct net_device *netdev = pci_get_drvdata(pdev);
	struct igb_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;

	printk("%s > \n",__func__);
	iounmap(hw->hw_addr);
	if (hw->flash_address)
		iounmap(hw->flash_address);
	pci_release_selected_regions(pdev,
	                    pci_select_bars(pdev, IORESOURCE_MEM));

	free_netdev(netdev);

	pci_disable_device(pdev);
}

static struct pci_driver igb_driver_diag = {
	.name     = igb_driver_name,
	.id_table = igb_pci_tbl_diag,
	.probe    = igb_probe_diag,
	.remove   = __devexit_p(igb_remove_diag),
};

static int __init igb_init_module_diag(void)
{
	int ret;

	printk(KERN_INFO "%s - version %s\n",
	       igb_driver_string, igb_driver_version);

	printk(KERN_INFO "%s - version %s diag\n",
	igb_driver_string, igb_driver_version);
	ret = pci_register_driver(&igb_driver_diag);
	return ret;
}
module_init(igb_init_module_diag);

static void __exit igb_exit_module_diag(void)
{
	pci_unregister_driver(&igb_driver_diag);
}
module_exit(igb_exit_module_diag);


MODULE_AUTHOR("Intel Corporation, <e1000-devel@lists.sourceforge.net>");
MODULE_DESCRIPTION("Intel(R) Gigabit Ethernet Network Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

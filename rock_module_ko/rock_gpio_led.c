#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/sched.h>   //wake_up_process()
#include <linux/kthread.h> //kthread_create()
#include <linux/err.h> //IS_ERR()

#if 0

#define DEVICE_COUNT 6

#define DRIVER_NAME "gpio_lq"
#define DEVICE_NAME "gpio_lq"
#define DEV_NODE_NAME "rock_module_node"

#define KEY_HOME             EXYNOS4_GPX1(1)
#define KEY_BACK             EXYNOS4_GPX1(2)
#define KEY_SLEEP              EXYNOS4_GPX3(3)
#define KEY_VOLUP         EXYNOS4_GPX2(1)
#define KEY_VOLDOWN  EXYNOS4_GPX2(0)

#define NUM_INT_KEY_HOME     IRQ_EINT(9)
#define NUM_INT_KEY_BACK              IRQ_EINT(10)
#define NUM_INT_KEY_SLEEP           IRQ_EINT(27)
#define NUM_INT_KEY_VOLUP         IRQ_EINT(17)
#define NUM_INT_KEY_VOLDOWN  IRQ_EINT(16)

#define LED1 EXYNOS4_GPL2(0)
#define LED2 EXYNOS4_GPK1(1)

static int Leds[] = {
    LED1,
    LED2,
};

static int Gpios[] = {
    KEY_HOME,
    KEY_BACK,
    KEY_SLEEP,
    KEY_VOLUP,
    KEY_VOLDOWN,
};

static int Num_Int_Gpios[] = {
    NUM_INT_KEY_HOME,
    NUM_INT_KEY_BACK,
    NUM_INT_KEY_SLEEP,
    NUM_INT_KEY_VOLUP,
    NUM_INT_KEY_VOLDOWN,
};

static DECLARE_COMPLETION(exit_completion); 

#define LED_NUM        ARRAY_SIZE(Leds)
#define GPIO_NUM        ARRAY_SIZE(Gpios)

static struct class * rock_module_leds_class;
static int g_iGpios_Leds_Dev_Count = 0;
static struct rock_module_leds_device  * g_pGpios_Leds_Devs[DEVICE_COUNT];

typedef  irqreturn_t (*pFunc)(int irq, void *dev_id);

static struct task_struct *my_kernelthread_task;

//ring buffer  start =============================================
#define RING_BUFFER_LENGTH (4096 * 4)
#define FULL     (-1)
#define EMPTY  (-2)
#define NORMAL  (0)

struct ring_buffer 
{
    void * pMemKernelBase;
    void * pMemUserBase;
    long long WriteIndex;
    long long ReadIndex;
    unsigned char status;
};

struct rock_module_leds_device 
{
    struct cdev cdev;
    struct device * pdevice;
    struct ring_buffer * pRingBuffer;
};

void init_ring_buffer(struct ring_buffer * pRingBuffer)
{
    if (NULL == pRingBuffer)
    {
        return EINVAL;
    }
    pRingBuffer->pMemKernelBase = NULL;
    pRingBuffer->ReadIndex = 0;
    pRingBuffer->WriteIndex = 0;
    pRingBuffer->status = EMPTY;
}

char push_ring_buffer(struct ring_buffer * pRingBuffer,unsigned char data)
{
    if(pRingBuffer->status == FULL)
    {
        return FULL;
    }
    *(char *)((char *)pRingBuffer->pMemKernelBase + pRingBuffer->WriteIndex++) = data;
    if(pRingBuffer->WriteIndex == RING_BUFFER_LENGTH)
    {
        pRingBuffer->WriteIndex = 0;
    }
    if(pRingBuffer->WriteIndex == pRingBuffer->ReadIndex)
    {
        pRingBuffer->status = FULL;
        return FULL;
    }
    else
    {
        pRingBuffer->status = NORMAL;
    }
    return NORMAL;
}

char pop_ring_buffer(struct ring_buffer * pRingBuffer,char * pRet)
{
    if(pRingBuffer->status == EMPTY)
    {
        return  EMPTY;
    }
    *pRet =  *((char *)pRingBuffer->pMemKernelBase + pRingBuffer->ReadIndex++);
    if(pRingBuffer->ReadIndex == RING_BUFFER_LENGTH)
    {
        pRingBuffer->ReadIndex = 0;
    }
    if(pRingBuffer->ReadIndex == pRingBuffer->WriteIndex)
    {
        pRingBuffer->status = EMPTY;
    }
    else
    {
        pRingBuffer->status = NORMAL;
    }
    return NORMAL;
}

//ring buffer  end =============================================

static irqreturn_t eint9_KEY_HOME_interrupt(int irq, void *dev_id)
{
        printk("%s(%d)\n", __FUNCTION__, __LINE__);
        if (gpio_get_value(Leds[0]))
        {
             gpio_set_value(Leds[0], 0);           
        }
        else
        {
              gpio_set_value(Leds[0], 1);              
        }
        return IRQ_HANDLED;
}
static irqreturn_t eint10_KEY_BACK_interrupt(int irq, void *dev_id) 
{
        printk("%s(%d)\n", __FUNCTION__, __LINE__);
        if (gpio_get_value(Leds[1]))
        {
             gpio_set_value(Leds[1], 0);           
        }
        else
        {
             gpio_set_value(Leds[1], 1);              
        }
        return IRQ_HANDLED;
}
static irqreturn_t eint27_KEY_SLEEP_interrupt(int irq, void *dev_id) 
{
        printk("%s(%d)\n", __FUNCTION__, __LINE__);
        return IRQ_HANDLED;
}
static irqreturn_t eint17_KEY_VOLUP_interrupt(int irq, void *dev_id) 
{
        printk("%s(%d)\n", __FUNCTION__, __LINE__);
        return IRQ_HANDLED;
}

static irqreturn_t eint16_KEY_VOLDOWN_interrupt(int irq, void *dev_id) 
{
        printk("%s(%d)\n", __FUNCTION__, __LINE__);
        return IRQ_HANDLED;
}

static pFunc pIntFunction[] = {
    eint9_KEY_HOME_interrupt,
    eint10_KEY_BACK_interrupt,
    eint27_KEY_SLEEP_interrupt,
    eint17_KEY_VOLUP_interrupt,
    eint16_KEY_VOLDOWN_interrupt,
};

static char * IntNameString[] = {
    "eint9_KEY_HOME_interrupt",
    "eint10_KEY_BACK_interrupt",
    "eint27_KEY_SLEEP_interrupt",
    "eint17_KEY_VOLUP_interrupt",
    "eint16_KEY_VOLDOWN_interrupt",
};

//using method: echo "123456">ctl_led0 ==>  ctl_leds:123456  count=7  buf[0]=1
//cat ctl_led0  ==> led 0: 1
static ssize_t ctl_leds(struct device * pdevice,struct device_attribute *attr,const char *buf,size_t count)
{
    struct rock_module_leds_device  *pDev = NULL;

    pDev = (struct rock_module_leds_device  *)(pdevice->platform_data);

    printk(KERN_EMERG"ctl_leds:%s count=%d  buf[0]=%c\n",buf,count,buf[0]);

    if (0 == strcmp(attr->attr.name,"ctl_led0"))
    {
        if('0' == buf[0])
        {
            gpio_set_value(Leds[0], 0);    
        }
        else if('1' == buf[0])
        {
            gpio_set_value(Leds[0], 1);                
        }
    }
    else  if (0 == strcmp(attr->attr.name,"ctl_led1"))
    {
        if('0' == buf[0])
        {
            gpio_set_value(Leds[1], 0);    
        }
        else if('1' == buf[0])
        {
            gpio_set_value(Leds[1], 1);                
        }
    }
    return count;
}

static ssize_t leds_show(struct device * pdevice,struct device_attribute *attr,const char *buf,size_t count)
{
    struct rock_module_leds_device  *pDev = NULL;

    pDev = (struct rock_module_leds_device  *)(pdevice->platform_data);

    if (0 == strcmp(attr->attr.name,"ctl_led0"))
    {
        printk(KERN_EMERG"led 0: %d\n",gpio_get_value(Leds[0]));
    }
    else  if (0 == strcmp(attr->attr.name,"ctl_led1"))
    {
        printk(KERN_EMERG"led 1: %d\n",gpio_get_value(Leds[1]));        
    }
    return 0;
}

static ssize_t kernel_memory_show(struct device * pdevice,struct device_attribute *attr,const char *buf,size_t count)
{
    struct rock_module_leds_device  *pDev = NULL;

    pDev = (struct rock_module_leds_device  *)(pdevice->platform_data);
    
    if (0 == strcmp(attr->attr.name,"ctl_led0"))
    {
        printk(KERN_EMERG"led 0: %d\n",gpio_get_value(Leds[0]));
    }
    return 0;
}

static ssize_t kernel_memory_write(struct device * pdevice,struct device_attribute *attr,const char *buf,size_t count)
{
    struct rock_module_leds_device  *pDev = NULL;

    pDev = (struct rock_module_leds_device  *)(pdevice->platform_data);

    if (0 == strcmp(attr->attr.name,"ctl_led0"))
    {
        printk(KERN_EMERG"led 0: %d\n",gpio_get_value(Leds[0]));
    }
    return 0;
}

static struct device_attribute  rock_module_leds_attribute[] = {
    __ATTR(ctl_led0,0777,leds_show,ctl_leds),
    __ATTR(ctl_led1,0777,leds_show,ctl_leds),
    __ATTR(kernel_memory,0777,kernel_memory_show,kernel_memory_write),
};

int Kernel_Monitor_Thread1(void * arg)
{
    daemonize("rocklee-thread1");     

    complete (&exit_completion);

    printk(KERN_EMERG"Kernel_Monitor_Thread1  current->pid=%d\n",current->pid); 

    while(1)
    {
        //printk(KERN_EMERG"Kernel_Monitor_Thread is running~~~\n");
        msleep(2000);
    }
}

int Kernel_Monitor_Thread2(void * arg)
{
    printk(KERN_EMERG"Kernel_Monitor_Thread2  current->pid=%d\n",current->pid); 

    while(1)
    {
        //printk(KERN_EMERG"Kernel_Monitor_Thread is running~~~\n");
        msleep(2000);
    }
}

static long rock_module_fops_ioctl( struct file *pfile, unsigned int cmd, unsigned long arg)
{
    struct rock_module_leds_device  *pDev = NULL;
    
    printk(KERN_EMERG"cmd is %d,arg is %d\n",cmd,arg);

    pDev = (struct rock_module_leds_device  *)pfile->private_data;
    
    switch(cmd)
    {
        case 0:
        case 1:
            if (arg > LED_NUM)
            {
                return -EINVAL;
            }
            gpio_set_value(Leds[arg], cmd);
            break;
        case 3: /*get  rock_module_leds_device of this file , arg is a pointer of user address*/ 
            copy_to_user((void *)arg,pDev,sizeof(struct rock_module_leds_device));
            break;
        default:
            return -EINVAL;
    }
    return 0;
}

static int rock_module_fops_release(struct inode *inode, struct file *pfile)
{
    int i = 0;
    struct rock_module_leds_device  *pDev = NULL;
    
    printk(KERN_EMERG "gpios release\n");
    
    pDev = (struct rock_module_leds_device  *)pfile->private_data;

    for(i=0; i < RING_BUFFER_LENGTH/PAGE_SIZE; i++)
    {
        ClearPageReserved(virt_to_page(pDev->pRingBuffer->pMemKernelBase + i * PAGE_SIZE));
    }
    
    free_pages((unsigned long)pDev->pRingBuffer->pMemKernelBase,get_order(RING_BUFFER_LENGTH));
    kfree(pDev->pRingBuffer);

    pDev->pRingBuffer->pMemKernelBase = NULL;
    pDev->pRingBuffer->pMemUserBase = NULL;
    pDev->pRingBuffer  = NULL;

    return 0;
}

static int rock_module_fops_open(struct inode *inode, struct file *pfile)
{
    int i = 0;
    int ret = 0;
    pid_t pid;
    struct rock_module_leds_device  *pDev = NULL;

    printk(KERN_EMERG "rock_module_fops_open...\n");

    for(i = 0; i < DEVICE_COUNT ; i++)
    {
        if((NULL != g_pGpios_Leds_Devs[i]) && (g_pGpios_Leds_Devs[i]->pdevice->devt  == inode->i_rdev))
        {
            pDev = g_pGpios_Leds_Devs[i];
            printk(KERN_EMERG "rock_module_fops_open  g_iGpios_Leds_Dev_Count =%d  inode->i_rdev=0x%x\n",i,inode->i_rdev);
            break;
        }
    }
    if(DEVICE_COUNT == i)
    {
        printk(KERN_EMERG "mismatch device\n");
        return -EINVAL;
    }
    if(NULL == pDev->pRingBuffer)
    {
        pDev->pRingBuffer = kmalloc(sizeof(*pDev->pRingBuffer),GFP_KERNEL);
        if(NULL == pDev->pRingBuffer)
        {
            printk(KERN_EMERG"kmalloc    failed \n");    
            return -ENOMEM;
        }
        init_ring_buffer(pDev->pRingBuffer);
        pDev->pRingBuffer->pMemKernelBase  = (void *) __get_free_pages(__GFP_DMA,get_order(RING_BUFFER_LENGTH));
        printk(KERN_EMERG"__get_free_pages    get_order(%d)=%d  PAGE_SIZE=%d \n",RING_BUFFER_LENGTH,get_order(RING_BUFFER_LENGTH),PAGE_SIZE);
        if(NULL == pDev->pRingBuffer->pMemKernelBase)
        {
            kfree(pDev->pRingBuffer);
            printk(KERN_EMERG"__get_free_pages    failed \n");    
            return -ENOMEM;
        }
        pfile->private_data = pDev;
        for(i=0; i < RING_BUFFER_LENGTH/PAGE_SIZE; i++)
        {
            SetPageReserved(virt_to_page(pDev->pRingBuffer->pMemKernelBase + i * PAGE_SIZE));
        }

        printk(KERN_EMERG"do_mmap...\n"); 
        if(do_mmap(pfile,0,RING_BUFFER_LENGTH,PROT_READ | PROT_WRITE,MAP_FILE | MAP_SHARED,0) == 0)
        {
            printk(KERN_EMERG"do_mmap  failed \n");
            return -ENOMEM;
        }
        //kernel_thread (int (*fn)(void *), void *arg, unsigned long flags)
        pid = kernel_thread(Kernel_Monitor_Thread1, NULL, CLONE_FS | CLONE_FILES | SIGCHLD); 
        if (pid < 0)
        {
            printk(KERN_EMERG"kernel_thread failed pid=%d\n",pid);             
        }
        else
        {
            wait_for_completion(&exit_completion);
        }
        printk(KERN_EMERG"rock_module_fops_open  current->pid=%d\n",current->pid); 

        my_kernelthread_task = kthread_create(Kernel_Monitor_Thread2, NULL, "rocklee-thread2");
        if(IS_ERR(my_kernelthread_task))
        {
            int err;
            printk("Unable to start kernel thread./n");
            err = PTR_ERR(my_kernelthread_task);
            my_kernelthread_task = NULL;
            return err;
        }
        wake_up_process(my_kernelthread_task);
    }
    else
    {
        printk(KERN_EMERG"rock_module_fops_open:already open \n");
    }
    return 0;
}

static int rock_module_fops_write(struct file * pfile, char __user * buffer, size_t count, loff_t  * ppos)
{
    int rc = 0;
    int i = 0;
    struct rock_module_leds_device  *pDev = NULL;
    void * pMem = NULL;
    
    printk(KERN_EMERG "\nrock_module_fops_write  buffer: %s  count=%d\n",buffer,count);

    pDev = (struct rock_module_leds_device  *)pfile->private_data;
    if(NULL == pDev) 
        return -EINVAL;

    pMem = kmalloc(sizeof(count),GFP_KERNEL); //MAX SIZE   128K -16
    if(NULL == pMem)
    {
        printk(KERN_EMERG"kmalloc    failed \n");    
        return -ENOMEM;
    }
    
    if( copy_from_user(pMem,buffer,count))
    {
        printk(KERN_EMERG "copy_from_user  failed\n");
        return  -EFAULT;
    }
    for(i = 0; i < count; i++)
    {
        rc = push_ring_buffer(pDev->pRingBuffer,*((char *)pMem + i));
        if(rc)
        {
            kfree(pMem);
            *ppos += i;
            printk(KERN_EMERG "ring buffer is full: %d *ppos=%d\n",i,*ppos);
            return -ENOMEM;
        }
    }
    kfree(pMem);
    *ppos += count;
    return 0;
}

static int rock_module_fops_read(struct file * pfile, char __user * buffer, size_t count, loff_t  * ppos)
{
    int rc = 0;
    int i = 0;
    struct rock_module_leds_device  *pDev = NULL;
    void * pMem = NULL;
    
    printk(KERN_EMERG "\nrock_module_fops_read  buffer: %s  count=%d\n",buffer,count);

    pDev = (struct rock_module_leds_device  *)pfile->private_data;
    if(NULL == pDev) 
        return -EINVAL;

    pMem = kmalloc(sizeof(count),GFP_KERNEL); //MAX SIZE   128K -16
    if(NULL == pMem)
    {
        printk(KERN_EMERG"kmalloc    failed \n");    
        return -ENOMEM;
    }

    for(i = 0; i < count; i++)
    {
        rc  = pop_ring_buffer(pDev->pRingBuffer,(char *)pMem+i);
        if(rc)
        {    
            kfree(pMem);
            *ppos += i;
            printk(KERN_EMERG "ring buffer is empty: %d *ppos=%d\n",i,*ppos);
            return -ENOMEM;
        }
    }

    if( copy_to_user(buffer,pMem,i))
    {
        printk(KERN_EMERG "copy_to_user  failed\n");
        return    -EFAULT;
    }

    kfree(pMem);
    *ppos += i;
    return 0;
}

//int (*mmap) (struct file *, struct vm_area_struct *);
static int rock_module_fops_mmap (struct file *pfile, struct vm_area_struct *vma)
{
    int rc = 0;
    int i = 0;
    struct rock_module_leds_device  *pDev = NULL;
    printk(KERN_EMERG "rock_module_fops_mmap\n");
    
    pDev = (struct rock_module_leds_device  *)pfile->private_data;

    if(remap_pfn_range(vma,vma->vm_start,virt_to_phys(pDev->pRingBuffer->pMemKernelBase) >> PAGE_SHIFT,RING_BUFFER_LENGTH,PAGE_SHARED) != 0)
    {
        printk(KERN_EMERG "remap_pfn_range failed\n");        
        return -ENOMEM;
    }
    pDev->pRingBuffer->pMemUserBase = vma->vm_start;
    return 0;
}

static struct file_operations fops_rock_module_leds = {
    .owner = THIS_MODULE,
    .open = rock_module_fops_open,
    .release = rock_module_fops_release,
    .unlocked_ioctl = rock_module_fops_ioctl,
    .mmap = rock_module_fops_mmap,
    .write = rock_module_fops_write,
    .read = rock_module_fops_read,
};

static int rock_module_probe(struct platform_device *pdv)
{
    int ret,i;
    struct rock_module_leds_device  *pDev = NULL;
    dev_t devt;

    printk(KERN_EMERG "rock_module_probe  initializing~~~\n");

    ret = alloc_chrdev_region(&devt,0,1,"rock_module_leds");
    if (ret)
    {
        printk(KERN_EMERG"alloc_chrdev_region  failed, ret = %d\n", ret);
        goto exit;
    }

    printk(KERN_EMERG"MAJOR(devt)=%d  MINOR(devt)=%d \n", MAJOR(devt),MINOR(devt));

    pDev = kmalloc(sizeof(*pDev),GFP_KERNEL);
    if (NULL == pDev)
    {
        ret = -ENOMEM;
        printk(KERN_EMERG"kmalloc  failed \n");    
        goto exit_class;
    }
    memset(pDev,0,sizeof(*pDev));

    rock_module_leds_class = class_create(THIS_MODULE,"rock_module_leds_class"); //create device class, ready for the next steps
    if(NULL == rock_module_leds_class)
    {
        ret = -EBUSY;
        printk(KERN_EMERG"class_create  failed \n");    
        goto exit_class;
    }
    cdev_init(&pDev->cdev,&fops_rock_module_leds); //character device init  with  fops 
    pDev->cdev.owner = THIS_MODULE; //character device init  
    ret = cdev_add(&pDev->cdev,devt,1);//insert cdev using devt  into system 
    pDev->pdevice = device_create(rock_module_leds_class,NULL,devt,pDev,DEV_NODE_NAME);//create device node belong to device class(rock_module_leds_class)
    if(NULL == pDev->pdevice)
    {
        ret = -EBUSY;
        printk(KERN_EMERG"device_create  failed \n"); 
        goto exit_cdev;
    }

    pDev->pdevice->platform_data = pDev;  //for passing paramter  to other function
    pdv->dev.platform_data = pDev;  //for passing paramter  to other function
    
    //create sysfs ==>/sys/
    for(i = 0; i < ARRAY_SIZE(rock_module_leds_attribute); i++)
    {
        ret = device_create_file(pDev->pdevice,&rock_module_leds_attribute[i]); //add sysfs entry 
        if (ret)
        {
            printk(KERN_EMERG"add sysfs entry  failed, ret = %d\n", ret);
        }
    }
    for(i=0; i<LED_NUM; i++)
    {
        ret = gpio_request(Leds[i], "LED");
        if (ret) {
            printk("%s: request GPIO %d for LED failed, ret = %d\n", DRIVER_NAME,i, ret);
            goto exit_unavailable;
        }
        else{
            s3c_gpio_cfgpin(Leds[i], S3C_GPIO_OUTPUT);
            gpio_set_value(Leds[i], 1);
        }
    }
    for(i=0; i<GPIO_NUM; i++)
    {
            ret = gpio_request(Gpios[i], "GPIO");
        if (ret) {
            printk("%s: request GPIO %d for GPIO failed, ret = %d\n", DRIVER_NAME,i, ret);
            goto exit_unavailable;
        }
        else{
                s3c_gpio_cfgpin(Gpios[i], S3C_GPIO_SFN(0xF));
                s3c_gpio_setpull(Gpios[i], S3C_GPIO_PULL_UP);
                ret = request_irq(Num_Int_Gpios[i], pIntFunction[i],
                                IRQ_TYPE_EDGE_FALLING /*IRQF_TRIGGER_FALLING*/, IntNameString[i], pdv);
                if (ret < 0) {
                        printk("Request IRQ %d failed, %d\n", Num_Int_Gpios[i], ret);
                        goto exit_unavailable;
                }
        }
    }    
    g_pGpios_Leds_Devs[g_iGpios_Leds_Dev_Count] = pDev;
    g_iGpios_Leds_Dev_Count ++;
    
    return 0;
    
exit_unavailable:
    device_destroy(rock_module_leds_class,pDev->pdevice->devt);
exit_cdev:
    cdev_del(&pDev->cdev);
exit_class:
    unregister_chrdev_region(pDev->pdevice->devt,1);
exit:
    return ret;
}

static int rock_module_remove(struct platform_device *pdv)
{
    //this is xxx_remove for driver,not for device,so don't do anythind about  device here
    
    int i;
    struct rock_module_leds_device  *pDev = NULL;


    pDev = (struct rock_module_leds_device  *)(pdv->dev.platform_data);
        
    printk(KERN_EMERG "\tremove\n");
    for(i=0; i<LED_NUM; i++)
    {
        gpio_free(Leds[i]);
    }
    for(i=0; i<GPIO_NUM; i++)
    {
        gpio_free(Gpios[i]);
        free_irq(Num_Int_Gpios[i],pdv); 
    }
    for(i = 0; i < ARRAY_SIZE(rock_module_leds_attribute); i++)
    {
        device_remove_file(pDev->pdevice,&rock_module_leds_attribute[i]); //rm sysfs entry 
    }
    device_destroy(rock_module_leds_class,pDev->pdevice->devt);
    class_destroy(rock_module_leds_class);
    unregister_chrdev_region(pDev->pdevice->devt,1);
    cdev_del(&pDev->cdev);
    memset(pDev,0,sizeof(*pDev));
    kfree(pDev);
    g_pGpios_Leds_Devs[g_iGpios_Leds_Dev_Count] = NULL;
    g_iGpios_Leds_Dev_Count --;
    return 0;
}

static void rock_module_shutdown(struct platform_device *pdv)
{
    ;
}

static int rock_module_suspend(struct platform_device *pdv,pm_message_t pmt)
{
    return 0;
}

static int rock_module_resume(struct platform_device *pdv)
{
    return 0;
}

struct platform_driver rock_module_driver = {
    .probe = rock_module_probe,
    .remove = rock_module_remove,
    .shutdown = rock_module_shutdown,
    .suspend = rock_module_suspend,
    .resume = rock_module_resume,
    .driver = {
        .name = DRIVER_NAME,
        .owner = THIS_MODULE,
    }
};

void    rock_module_device_release(struct device *dev)
{
    printk(KERN_EMERG "rock_module_device_release~~~~\n");
}

struct platform_device rock_module_device = {
    .name = DEVICE_NAME,
    .id = -1,
    .dev.release = rock_module_device_release,
};

static int rock_module_init(void)
{
    int DriverState;
    int DeviceState; 
    int ret;
    
    printk(KERN_EMERG "rock_module_init~~~~\n");
    
    g_iGpios_Leds_Dev_Count = 0;
    memset(g_pGpios_Leds_Devs,0,sizeof(g_pGpios_Leds_Devs));

    DriverState = platform_driver_register(&rock_module_driver);
    printk(KERN_EMERG "\tDriverState is %d\n",DriverState);

    DeviceState = platform_device_register(&rock_module_device);
    printk(KERN_EMERG "\tDeviceState is %d\n",DeviceState);

    return 0;
}

static void rock_module_exit(void)
{
    printk(KERN_EMERG "rock_module_exit~~~~\n");    
    platform_driver_unregister(&rock_module_driver);
    platform_device_unregister(&rock_module_device);
}

#endif

#if 0

static int rock_module_init(void)
{
    printk(KERN_EMERG "rock_module_init~~~~\n");
    return 0;
}

static void rock_module_exit(void)
{
    printk(KERN_EMERG "rock_module_exit~~~~\n");    
}

module_init(rock_module_init);
module_exit(rock_module_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("ROCKLEE");
#endif 

#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/basic_mmio_gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <asm-generic/bug.h>

enum mxc_gpio_hwtype {
	IMX1_GPIO,	/* runs on i.mx1 */
	IMX21_GPIO,	/* runs on i.mx21 and i.mx27 */
	IMX31_GPIO,	/* runs on i.mx31 */
	IMX35_GPIO,	/* runs on all other i.mx */
};

/* device type dependent stuff */
struct mxc_gpio_hwdata {
	unsigned dr_reg;
	unsigned gdir_reg;
	unsigned psr_reg;
	unsigned icr1_reg;
	unsigned icr2_reg;
	unsigned imr_reg;
	unsigned isr_reg;
	int edge_sel_reg;
	unsigned low_level;
	unsigned high_level;
	unsigned rise_edge;
	unsigned fall_edge;
};

struct mxc_gpio_port {
	struct list_head node;
	void __iomem *base;
	int irq;
	int irq_high;
	struct irq_domain *domain;
	struct bgpio_chip bgc;
	u32 both_edges;
};

static struct mxc_gpio_hwdata imx1_imx21_gpio_hwdata = {
	.dr_reg		= 0x1c,
	.gdir_reg	= 0x00,
	.psr_reg	= 0x24,
	.icr1_reg	= 0x28,
	.icr2_reg	= 0x2c,
	.imr_reg	= 0x30,
	.isr_reg	= 0x34,
	.edge_sel_reg	= -EINVAL,
	.low_level	= 0x03,
	.high_level	= 0x02,
	.rise_edge	= 0x00,
	.fall_edge	= 0x01,
};

static struct mxc_gpio_hwdata imx31_gpio_hwdata = {
	.dr_reg		= 0x00,
	.gdir_reg	= 0x04,
	.psr_reg	= 0x08,
	.icr1_reg	= 0x0c,
	.icr2_reg	= 0x10,
	.imr_reg	= 0x14,
	.isr_reg	= 0x18,
	.edge_sel_reg	= -EINVAL,
	.low_level	= 0x00,
	.high_level	= 0x01,
	.rise_edge	= 0x02,
	.fall_edge	= 0x03,
};

static struct mxc_gpio_hwdata imx35_gpio_hwdata = {
	.dr_reg		= 0x00,
	.gdir_reg	= 0x04,
	.psr_reg	= 0x08,
	.icr1_reg	= 0x0c,
	.icr2_reg	= 0x10,
	.imr_reg	= 0x14,
	.isr_reg	= 0x18,
	.edge_sel_reg	= 0x1c,
	.low_level	= 0x00,
	.high_level	= 0x01,
	.rise_edge	= 0x02,
	.fall_edge	= 0x03,
};

static enum mxc_gpio_hwtype mxc_gpio_hwtype;
static struct mxc_gpio_hwdata *mxc_gpio_hwdata;

#define GPIO_DR			(mxc_gpio_hwdata->dr_reg)
#define GPIO_GDIR		(mxc_gpio_hwdata->gdir_reg)
#define GPIO_PSR		(mxc_gpio_hwdata->psr_reg)
#define GPIO_ICR1		(mxc_gpio_hwdata->icr1_reg)
#define GPIO_ICR2		(mxc_gpio_hwdata->icr2_reg)
#define GPIO_IMR		(mxc_gpio_hwdata->imr_reg)
#define GPIO_ISR		(mxc_gpio_hwdata->isr_reg)
#define GPIO_EDGE_SEL		(mxc_gpio_hwdata->edge_sel_reg)

#define GPIO_INT_LOW_LEV	(mxc_gpio_hwdata->low_level)
#define GPIO_INT_HIGH_LEV	(mxc_gpio_hwdata->high_level)
#define GPIO_INT_RISE_EDGE	(mxc_gpio_hwdata->rise_edge)
#define GPIO_INT_FALL_EDGE	(mxc_gpio_hwdata->fall_edge)
#define GPIO_INT_BOTH_EDGES	0x4

static struct platform_device_id mxc_gpio_devtype[] = {
	{
		.name = "imx1-gpio",
		.driver_data = IMX1_GPIO,
	}, {
		.name = "imx21-gpio",
		.driver_data = IMX21_GPIO,
	}, {
		.name = "imx31-gpio",
		.driver_data = IMX31_GPIO,
	}, {
		.name = "imx35-gpio",
		.driver_data = IMX35_GPIO,
	}, {
		/* sentinel */
	}
};

static const struct of_device_id mxc_gpio_dt_ids[] = {
	{ .compatible = "fsl,imx1-gpio", .data = &mxc_gpio_devtype[IMX1_GPIO], },
	{ .compatible = "fsl,imx21-gpio", .data = &mxc_gpio_devtype[IMX21_GPIO], },
	{ .compatible = "fsl,imx31-gpio", .data = &mxc_gpio_devtype[IMX31_GPIO], },
	{ .compatible = "fsl,imx35-gpio", .data = &mxc_gpio_devtype[IMX35_GPIO], },
	{ /* sentinel */ }
};

/*
 * MX2 has one interrupt *for all* gpio ports. The list is used
 * to save the references to all ports, so that mx2_gpio_irq_handler
 * can walk through all interrupt status registers.
 */
static LIST_HEAD(mxc_gpio_ports);

/* Note: This driver assumes 32 GPIOs are handled in one register */

static int gpio_set_irq_type(struct irq_data *d, u32 type)
{
	struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);
	struct mxc_gpio_port *port = gc->private;
	u32 bit, val;
	u32 gpio_idx = d->hwirq;
	u32 gpio = port->bgc.gc.base + gpio_idx;
	int edge;
	void __iomem *reg = port->base;

	port->both_edges &= ~(1 << gpio_idx);
	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		edge = GPIO_INT_RISE_EDGE;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		edge = GPIO_INT_FALL_EDGE;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		if (GPIO_EDGE_SEL >= 0) {
			edge = GPIO_INT_BOTH_EDGES;
		} else {
			val = gpio_get_value(gpio);
			if (val) {
				edge = GPIO_INT_LOW_LEV;
				pr_debug("mxc: set GPIO %d to low trigger\n", gpio);
			} else {
				edge = GPIO_INT_HIGH_LEV;
				pr_debug("mxc: set GPIO %d to high trigger\n", gpio);
			}
			port->both_edges |= 1 << gpio_idx;
		}
		break;
	case IRQ_TYPE_LEVEL_LOW:
		edge = GPIO_INT_LOW_LEV;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		edge = GPIO_INT_HIGH_LEV;
		break;
	default:
		return -EINVAL;
	}

	if (GPIO_EDGE_SEL >= 0) {
		val = readl(port->base + GPIO_EDGE_SEL);
		if (edge == GPIO_INT_BOTH_EDGES)
			writel(val | (1 << gpio_idx),
				port->base + GPIO_EDGE_SEL);
		else
			writel(val & ~(1 << gpio_idx),
				port->base + GPIO_EDGE_SEL);
	}

	if (edge != GPIO_INT_BOTH_EDGES) {
		reg += GPIO_ICR1 + ((gpio_idx & 0x10) >> 2); /* lower or upper register */
		bit = gpio_idx & 0xf;
		val = readl(reg) & ~(0x3 << (bit << 1));
		writel(val | (edge << (bit << 1)), reg);
	}

	writel(1 << gpio_idx, port->base + GPIO_ISR);

	return 0;
}

static void mxc_flip_edge(struct mxc_gpio_port *port, u32 gpio)
{
	void __iomem *reg = port->base;
	u32 bit, val;
	int edge;

	reg += GPIO_ICR1 + ((gpio & 0x10) >> 2); /* lower or upper register */
	bit = gpio & 0xf;
	val = readl(reg);
	edge = (val >> (bit << 1)) & 3;
	val &= ~(0x3 << (bit << 1));
	if (edge == GPIO_INT_HIGH_LEV) {
		edge = GPIO_INT_LOW_LEV;
		pr_debug("mxc: switch GPIO %d to low trigger\n", gpio);
	} else if (edge == GPIO_INT_LOW_LEV) {
		edge = GPIO_INT_HIGH_LEV;
		pr_debug("mxc: switch GPIO %d to high trigger\n", gpio);
	} else {
		pr_err("mxc: invalid configuration for GPIO %d: %x\n",
		       gpio, edge);
		return;
	}
	writel(val | (edge << (bit << 1)), reg);
}

/* handle 32 interrupts in one status register */
static void mxc_gpio_irq_handler(struct mxc_gpio_port *port, u32 irq_stat)
{
	while (irq_stat != 0) {
		int irqoffset = fls(irq_stat) - 1;

		if (port->both_edges & (1 << irqoffset))
			mxc_flip_edge(port, irqoffset);

		generic_handle_irq(irq_find_mapping(port->domain, irqoffset));

		irq_stat &= ~(1 << irqoffset);
	}
}

/* MX1 and MX3 has one interrupt *per* gpio port */
static void mx3_gpio_irq_handler(u32 irq, struct irq_desc *desc)
{
	u32 irq_stat;
	struct mxc_gpio_port *port = irq_get_handler_data(irq);
	struct irq_chip *chip = irq_get_chip(irq);

	chained_irq_enter(chip, desc);

	irq_stat = readl(port->base + GPIO_ISR) & readl(port->base + GPIO_IMR);

	mxc_gpio_irq_handler(port, irq_stat);

	chained_irq_exit(chip, desc);
}

/* MX2 has one interrupt *for all* gpio ports */
static void mx2_gpio_irq_handler(u32 irq, struct irq_desc *desc)
{
	u32 irq_msk, irq_stat;
	struct mxc_gpio_port *port;
	struct irq_chip *chip = irq_get_chip(irq);

	chained_irq_enter(chip, desc);

	/* walk through all interrupt status registers */
	list_for_each_entry(port, &mxc_gpio_ports, node) {
		irq_msk = readl(port->base + GPIO_IMR);
		if (!irq_msk)
			continue;

		irq_stat = readl(port->base + GPIO_ISR) & irq_msk;
		if (irq_stat)
			mxc_gpio_irq_handler(port, irq_stat);
	}
	chained_irq_exit(chip, desc);
}

/*
 * Set interrupt number "irq" in the GPIO as a wake-up source.
 * While system is running, all registered GPIO interrupts need to have
 * wake-up enabled. When system is suspended, only selected GPIO interrupts
 * need to have wake-up enabled.
 * @param  irq          interrupt source number
 * @param  enable       enable as wake-up if equal to non-zero
 * @return       This function returns 0 on success.
 */
static int gpio_set_wake_irq(struct irq_data *d, u32 enable)
{
	struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);
	struct mxc_gpio_port *port = gc->private;
	u32 gpio_idx = d->hwirq;

	if (enable) {
		if (port->irq_high && (gpio_idx >= 16))
			enable_irq_wake(port->irq_high);
		else
			enable_irq_wake(port->irq);
	} else {
		if (port->irq_high && (gpio_idx >= 16))
			disable_irq_wake(port->irq_high);
		else
			disable_irq_wake(port->irq);
	}

	return 0;
}

static void __init mxc_gpio_init_gc(struct mxc_gpio_port *port, int irq_base)
{
	struct irq_chip_generic *gc;
	struct irq_chip_type *ct;

	gc = irq_alloc_generic_chip("gpio-mxc", 1, irq_base,
				    port->base, handle_level_irq);
	gc->private = port;

	ct = gc->chip_types;
	ct->chip.irq_ack = irq_gc_ack_set_bit;
	ct->chip.irq_mask = irq_gc_mask_clr_bit;
	ct->chip.irq_unmask = irq_gc_mask_set_bit;
	ct->chip.irq_set_type = gpio_set_irq_type;
	ct->chip.irq_set_wake = gpio_set_wake_irq;
	ct->regs.ack = GPIO_ISR;
	ct->regs.mask = GPIO_IMR;

	irq_setup_generic_chip(gc, IRQ_MSK(32), IRQ_GC_INIT_NESTED_LOCK,
			       IRQ_NOREQUEST, 0);
}

static void mxc_gpio_get_hw(struct platform_device *pdev)
{
	const struct of_device_id *of_id =
			of_match_device(mxc_gpio_dt_ids, &pdev->dev);
	enum mxc_gpio_hwtype hwtype;

	if (of_id)
		pdev->id_entry = of_id->data;
	hwtype = pdev->id_entry->driver_data;

	if (mxc_gpio_hwtype) {
		/*
		 * The driver works with a reasonable presupposition,
		 * that is all gpio ports must be the same type when
		 * running on one soc.
		 */
		BUG_ON(mxc_gpio_hwtype != hwtype);
		return;
	}

	if (hwtype == IMX35_GPIO)
		mxc_gpio_hwdata = &imx35_gpio_hwdata;
	else if (hwtype == IMX31_GPIO)
		mxc_gpio_hwdata = &imx31_gpio_hwdata;
	else
		mxc_gpio_hwdata = &imx1_imx21_gpio_hwdata;

	mxc_gpio_hwtype = hwtype;
}

static int mxc_gpio_to_irq(struct gpio_chip *gc, unsigned offset)
{
	struct bgpio_chip *bgc = to_bgpio_chip(gc);
	struct mxc_gpio_port *port =
		container_of(bgc, struct mxc_gpio_port, bgc);

	return irq_find_mapping(port->domain, offset);
}

static int mxc_gpio_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct mxc_gpio_port *port;
	struct resource *iores;
	int irq_base;
	int err;

        printk(KERN_EMERG "rock_module_gpio_probe~~~~\n");

	mxc_gpio_get_hw(pdev);

	port = devm_kzalloc(&pdev->dev, sizeof(*port), GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	iores = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	port->base = devm_ioremap_resource(&pdev->dev, iores);
	if (IS_ERR(port->base))
		return PTR_ERR(port->base);

	port->irq_high = platform_get_irq(pdev, 1);
	port->irq = platform_get_irq(pdev, 0);
	if (port->irq < 0)
		return port->irq;

	/* disable the interrupt and clear the status */
	writel(0, port->base + GPIO_IMR);
	writel(~0, port->base + GPIO_ISR);

	if (mxc_gpio_hwtype == IMX21_GPIO) {
		/*
		 * Setup one handler for all GPIO interrupts. Actually setting
		 * the handler is needed only once, but doing it for every port
		 * is more robust and easier.
		 */
		irq_set_chained_handler(port->irq, mx2_gpio_irq_handler);
	} else {
		/* setup one handler for each entry */
		irq_set_chained_handler(port->irq, mx3_gpio_irq_handler);
		irq_set_handler_data(port->irq, port);
		if (port->irq_high > 0) {
			/* setup handler for GPIO 16 to 31 */
			irq_set_chained_handler(port->irq_high,
						mx3_gpio_irq_handler);
			irq_set_handler_data(port->irq_high, port);
		}
	}

	err = bgpio_init(&port->bgc, &pdev->dev, 4,
			 port->base + GPIO_PSR,
			 port->base + GPIO_DR, NULL,
			 port->base + GPIO_GDIR, NULL, 0);
	if (err)
		goto out_bgio;

	port->bgc.gc.to_irq = mxc_gpio_to_irq;
	port->bgc.gc.base = (pdev->id < 0) ? of_alias_get_id(np, "gpio") * 32 :
					     pdev->id * 32;

	err = gpiochip_add(&port->bgc.gc);
	if (err)
		goto out_bgpio_remove;

	irq_base = irq_alloc_descs(-1, 0, 32, numa_node_id());
	if (irq_base < 0) {
		err = irq_base;
		goto out_gpiochip_remove;
	}

	port->domain = irq_domain_add_legacy(np, 32, irq_base, 0,
					     &irq_domain_simple_ops, NULL);
	if (!port->domain) {
		err = -ENODEV;
		goto out_irqdesc_free;
	}

	/* gpio-mxc can be a generic irq chip */
	mxc_gpio_init_gc(port, irq_base);

	list_add_tail(&port->node, &mxc_gpio_ports);

	return 0;

out_irqdesc_free:
	irq_free_descs(irq_base, 32);
out_gpiochip_remove:
	gpiochip_remove(&port->bgc.gc);
out_bgpio_remove:
	bgpio_remove(&port->bgc);
out_bgio:
	dev_info(&pdev->dev, "%s failed with errno %d\n", __func__, err);
	return err;
}

static struct platform_driver mxc_gpio_driver = {
	.driver		= {
		.name	= "gpio-mxc-rocklee",
		.of_match_table = mxc_gpio_dt_ids,
	},
	.probe		= mxc_gpio_probe,
	.id_table	= mxc_gpio_devtype,
};

static int rock_module_probe(struct platform_device *pdv)
{
    printk(KERN_EMERG "rock_module_probe  initializing~~~\n");
    return 0;
}

static int rock_module_remove(struct platform_device *pdv)
{
    return 0;
}

static void rock_module_shutdown(struct platform_device *pdv)
{
    ;
}

static int rock_module_suspend(struct platform_device *pdv, pm_message_t pmt)
{
    return 0;
}

static int rock_module_resume(struct platform_device *pdv)
{
    return 0;
}

struct platform_driver g_rock_module_driver = {
    .probe = rock_module_probe,
    .remove = rock_module_remove,
    .shutdown = rock_module_shutdown,
    .suspend = rock_module_suspend,
    .resume = rock_module_resume,
    .driver = {
        .name = "rock_module",
        .owner = THIS_MODULE,
    }
};

void rock_module_device_release(struct device *dev)
{
    printk(KERN_EMERG "rock_module_device_release~~~~\n");
}

struct platform_device g_rock_module_device = {
    .name = "rock_module",
    .id = -1,
    .dev.release = rock_module_device_release,
};


static int __init rock_module_init(void)
{
    int driver_state;
    int device_state; 
    
    printk(KERN_EMERG "rock_module_init~~~~\n");
     
    hello_fun();

    driver_state = platform_driver_register(&g_rock_module_driver);
    printk(KERN_EMERG "\tdriver_state is %d\n",driver_state);

    device_state = platform_device_register(&g_rock_module_device);
    printk(KERN_EMERG "\tdevice_state is %d\n",device_state);

    return 0;
}

static void __exit rock_module_exit(void)
{
    printk(KERN_EMERG "rock_module_exit~~~~\n");    
    platform_driver_unregister(&g_rock_module_driver);
    platform_device_unregister(&g_rock_module_device);
}

module_init(rock_module_init);
module_exit(rock_module_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("ROCKLEE");

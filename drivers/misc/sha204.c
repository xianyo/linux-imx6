#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/proc_fs.h>
#include <linux/regulator/consumer.h>

#include <linux/uaccess.h>
#include <linux/miscdevice.h>


#define SHA204_WRITE	0x01
#define SHA204_READ     0x03

#define _SHA204DEBUG_ 1

#define I2C_M_TEN		0x0010	/* this is a ten bit chip address */
#define I2C_M_RD		0x0001	/* read data, from slave to master */
#define I2C_M_STOP		0x8000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_NOSTART		0x4000	/* if I2C_FUNC_NOSTART */
#define I2C_M_REV_DIR_ADDR	0x2000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_IGNORE_NAK	0x1000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_NO_RD_ACK		0x0800	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_RECV_LEN		0x0400	/* length will be first received byte */


//struct regulator *sha204_regulor=NULL;
//static bool sha204_init_flag = false;

struct i2c_client *sha204_client = NULL;

struct my_i2c_msg {
	unsigned short addr;	/* slave address			*/
	unsigned short flags;
	unsigned short len;		/* msg length				*/
	unsigned char buf[64];		/* pointer to msg data			*/
};


static inline int sensor_i2c_master_wake(struct i2c_client *client, int flags, char *buf ,int count)
{
	int ret;
	struct i2c_adapter *adap=client->adapter;
	struct i2c_msg msg;

	memset(buf, 0, 32);

	msg.addr = 0x0;
	msg.flags = client->flags & I2C_M_TEN;
	msg.len = 3;
	msg.buf = (char *)buf;
	#if _SHA204DEBUG_
	printk("wake client addr is %d flags %x\n",client->addr, msg.flags);
	#endif
	ret = i2c_transfer(adap, &msg, 1);

	/* If everything went ok (i.e. 1 msg transmitted), return #bytes
	   transmitted, else error code. */
	return (ret == 1) ? count : ret;
}


static inline int sensor_i2c_master_send(struct i2c_client *client, int flags, const char *buf ,int count)
{
	int ret;
	struct i2c_adapter *adap=client->adapter;
	struct i2c_msg msg;

	msg.addr = client->addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.len = count;
	msg.buf = (char *)buf;
	#if _SHA204DEBUG_
	printk("write client addr is %d flags %x\n",client->addr, msg.flags);
	#endif
	
	ret = i2c_transfer(adap, &msg, 1);

	/* If everything went ok (i.e. 1 msg transmitted), return #bytes
	   transmitted, else error code. */
	return (ret == 1) ? count : ret;
}

static inline int sensor_i2c_master_recv(struct i2c_client *client, int flags, char *buf ,int count)
{
	struct i2c_adapter *adap=client->adapter;
	struct i2c_msg msg;
	int ret;

	msg.addr = client->addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.flags |= I2C_M_RD;
	msg.len = count;
	msg.buf = buf;
	#if _SHA204DEBUG_
	printk("read client addr is %d flags %x\n",client->addr, msg.flags);
	#endif
	ret = i2c_transfer(adap, &msg, 1);

	/* If everything went ok (i.e. 1 msg transmitted), return #bytes
	   transmitted, else error code. */
	return (ret == 1) ? count : ret;
}

static int sha204_write_reg(struct i2c_client *client, struct my_i2c_msg *packet)
{
	int ret;

	if (packet->addr == 0)
		ret = sensor_i2c_master_wake(client, packet->flags, packet->buf, packet->len);
	else
		ret = sensor_i2c_master_send(client, packet->flags, packet->buf, packet->len);

	if (ret < 0)
	{
		printk("sha204 write error %d\n",ret);
		return ret;
	}
	#if _SHA204DEBUG_
	printk("sha204 write %d\n",ret);
	#endif
	return 0;
}


static int sha204_read_reg(struct i2c_client *client, struct my_i2c_msg *packet)
{
	int ret;
	//unsigned char retval;

	ret = sensor_i2c_master_recv(client, packet->flags, packet->buf, packet->len);
	if (ret < 0)
	{
		printk(KERN_ERR "read ret error val %d\n", ret);
		return ret;
	}
	#if _SHA204DEBUG_
	printk("read ret %d\n", ret);
	#endif
	if (ret != packet->len)
		return -EIO;
	return ret;
}

static int sha204_open(struct inode *inode, struct file *file)
{
	//struct miscdevice *dev = file->private_data;

	//regulator_enable(sha204_regulor);
	//regulator_enable(NULL);


	printk("sha204_open\n");
	return 0;
}


unsigned int sha204_poll(struct file *file, struct poll_table_struct *wait)
{
	return 0;	
}

static int sha204_close(struct inode *inode, struct file *file)
{
	printk("sha204_close\n");
//	regulator_disable(sha204_regulor);

	return 0;
}


static int sha204_mac_verify(struct i2c_client *client);

static long sha204_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	
	void __user *argp = (void __user *)arg;
	struct my_i2c_msg packet;
	struct i2c_client *client ;
	long ret = 0;
	int res;
	int buf_size = sizeof(packet);
	
	switch (cmd) {
		case SHA204_WRITE:
			if (copy_from_user(&packet, (struct my_i2c_msg __user *)argp, buf_size)) {
				printk(KERN_ERR "Failed to copy from user\n");
				return -EFAULT;
			}
/*			printk("send data :\n");
			for(i = 0;i<packet.len;i++)
				printk("0x%x ",packet.buf[i]); */
				
			client = sha204_client;
			if (sha204_write_reg(client, &packet))
			{
				printk(KERN_ERR "i2c write error\n");
				return -EIO;
			}
			break;
		case SHA204_READ:
			if (copy_from_user(&packet, argp, buf_size)) {
				printk(KERN_ERR "Failed to copy from user\n");
				return -EFAULT;
			}
			client = sha204_client;
			res = sha204_read_reg(client, &packet);
			if (res < 0)
			{
				printk(KERN_ERR "i2c read error\n");
				return -EIO;
			}
			
/*			printk("read data :\n");
			for(i = 0;i<packet.len;i++)
				printk("0x%x ",packet.buf[i]); */
			
			if (copy_to_user((char *)arg, &packet, sizeof(packet)))
				ret = -EFAULT;				
			break;
	    default:
	        return -ENOIOCTLCMD;
	}

	return ret;
}

static struct file_operations sha204_fops = {
	.open 		= sha204_open,
	.read 		= NULL,
	.poll		= sha204_poll,
	.mmap		= NULL,
	.release 	= sha204_close,
	.unlocked_ioctl = sha204_ioctl,
};

static struct miscdevice sha204_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = "sha204",
	.fops = &sha204_fops,
};

static void atCRC( unsigned char  length, unsigned char  *data, unsigned char  *crc)
{
	unsigned char counter;
	unsigned short crc_register = 0;
	unsigned short polynom = 0x8005;
	unsigned char  shift_register;
	unsigned char  data_bit, crc_bit;

	for (counter = 0; counter < length; counter++) {
		for (shift_register = 0x01; shift_register > 0x00; shift_register <<= 1) {
			data_bit = (data[counter] & shift_register) ? 1 : 0;
			crc_bit = crc_register >> 15;
			crc_register <<= 1;
			if (data_bit != crc_bit)
				crc_register ^= polynom;
		}
	}
	crc[0] = (unsigned char)(crc_register & 0x00FF);
	crc[1] = (unsigned char)(crc_register >> 8);
}

unsigned char rightMac[32] = 
{
	0x6A, 0x59, 0xB0, 0xF8, 0x2B, 0x56, 0x79, 0x30, 0xA8, 0xC7, 0xA8, 0xAE, 0x18, 0xF9, 0x29, 0x54, 
	0x05, 0x68, 0x69, 0x90, 0x40, 0x25, 0xAC, 0x26, 0x6E, 0x5A, 0x4C, 0x82, 0xCD, 0xD3, 0xD3, 0x24
};


static int sha204_mac_verify(struct i2c_client *client)
{
	s32 ret = -1;
	char buf[40]={0};
	char buf_rev[40]={0};
    int i;
	buf[0]=0x03;
	buf[1]=39;
	
	buf[2]=0x08;
	buf[3]=0x0;
	buf[4]=0x1;
	buf[5]=0x00;
	
	for(i = 0; i < 32; i++)
		buf[6+i] = i;
	
	atCRC(37, &buf[1], &buf[38]);
	
	
	printk("%s: send data :\n", __func__);
	for(i = 0;i<40;i++)
		printk("0x%x ",buf[i]);

    ret = i2c_master_send(client,buf,40);

	 printk("\n%s: send %d\n", __func__, ret);
             
	 msleep(5);
	
    ret = i2c_master_recv(client, buf_rev, 35);

	printk("%s: receive data :%d\n",__func__, ret);	
	for(i = 0;i<35;i++)
		printk("0x%x ", buf_rev[i]);
	i = 0;
	if(ret == 35)
	{
		for(i = 0; i < 32; i++)
		{
			if(buf_rev[i + 1 ] != rightMac[i])
			{
				break;
			}
		}
	}
	if(i == 32)
     	return 0;
	else
		return -ENODEV;
}

static int sha204_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
	char buf[32] = {0};
	printk("%s adapter->nr %d\n",__func__,adapter->nr);
	
	msleep(3);
			
    sensor_i2c_master_wake(client,0,buf,1);
	msleep(50);
	if(sha204_mac_verify(client) == 0){
		strlcpy(info->type, "sha204", I2C_NAME_SIZE);
		return 0;
	}

	return -ENODEV;
}


static int sha204_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	struct i2c_board_info sha204_info;
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C | I2C_FUNC_SMBUS_WRITE_BYTE | I2C_FUNC_SMBUS_READ_BYTE_DATA))
	{
		printk(KERN_ERR "SHA204 functionality check failed.\n");
		return -EIO;
	}
	
	sha204_client = client;
	#if _SHA204DEBUG_
	printk("%s: addr 0x%x \n", __func__, sha204_client->addr);
	#endif

	ret = sha204_detect(sha204_client, &sha204_info);
	if(ret == 0){
		ret = misc_register(&sha204_miscdev);
		if (ret < 0)
		{
			printk(KERN_ERR "%s: sha204_register_device: error = %d\n", __func__, ret);
			return ret;
		}
		#if _SHA204DEBUG_
		printk("\npaxdebug, %s: device info: %s\n", __func__, sha204_info.type);
		#endif
	}else{
		printk(KERN_ERR "%s: No device:\"sha204\", error = %d\n",__func__, ret);
		return ret;
	}
	
	return 0;
}

int sha204_remove(struct i2c_client *client)
{
    return misc_deregister(&sha204_miscdev);
}

static const struct i2c_device_id sha204_id[] = {
	{ "atsha204", 0 },
	{ }	/* Terminating entry */
};

MODULE_DEVICE_TABLE(i2c, sha204_id);


static struct of_device_id sha204_match_table[] = {
	{ .compatible = "AT,atsha204", },
	{ },
};

//static const unsigned short normal_i2c[2] = {0xc8>>1, I2C_CLIENT_END};


static struct i2c_driver sha204_driver = {
	.class			= I2C_CLASS_SPD,
	.probe		= sha204_probe,
	.remove = sha204_remove,
	.id_table	= sha204_id,
	.driver	= {
		.name = "sha204",
		.of_match_table = sha204_match_table,
		.owner	= THIS_MODULE,
	},
	//.address_list = normal_i2c,
	.detect = sha204_detect,
};

static int __init sha204_init(void)
{
	printk("sha204_register init\n");
	return i2c_add_driver(&sha204_driver);
}

static void __exit sha204_exit(void)
{
	i2c_del_driver(&sha204_driver);
	printk("sha204_unregister\n");	
}


module_init(sha204_init);
module_exit(sha204_exit);

MODULE_LICENSE("GPL");

/****************************************************************************
 * config/mB_I2C/src/stm32_cdc.c
 * Character driver for a simple I2C capacitance to digital converter.
 *
 * Based on MS58XX driver.
 *
 *   Copyright (C) 2015 Omni Hoverboards Inc. All rights reserved.
 *   Author: Paul Alexander Patience <paul-a.patience@polymtl.ca>
 *   Updated by: Karim Keddam <karim.keddam@polymtl.ca>
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/cdc.h>

#include "mB_I2C.h"

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_CDC)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_CDC_I2C_FREQUENCY
#  define CONFIG_CDC_I2C_FREQUENCY I2C_SPEED_FAST
#endif

#define CDC_RAW_SAMPLE_SIZE			7	/* status byte + 6 data bytes*/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct cdc_dev_s
{
  FAR struct i2c_master_s       *i2c; /* I2C interface */
  uint8_t                       addr; /* I2C address */

  uint8_t	status_cdc;		/* cdc status*/
  uint32_t	cap;			/* cdc capacitance value */
  uint32_t	temp;			/* cdc temperature value */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C Helpers */

static int cdc_i2c_write(FAR struct cdc_dev_s *priv,
                        FAR uint8_t *buffer, int buflen);
static int cdc_i2c_read(FAR struct cdc_dev_s *priv,
                        FAR uint8_t *buffer, int buflen);
static int cdc_measure(FAR struct cdc_dev_s *priv);
static int cdc_init(FAR struct cdc_dev_s *priv);


/* Character Driver Methods */

static int     cdc_open(FAR struct file *filep);
static int     cdc_close(FAR struct file *filep);
static ssize_t cdc_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t cdc_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static int     cdc_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_fops =
{
  cdc_open,
  cdc_close,
  cdc_read,
  cdc_write,
  NULL,
  cdc_ioctl,
  NULL
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cdc_i2c_write
 *
 * Description:
 *   Write to the I2C device.
 *
 ****************************************************************************/

static int cdc_i2c_write(FAR struct cdc_dev_s *priv,
                        FAR uint8_t *buffer, int buflen)
{
  struct i2c_msg_s msg;
  int ret;

  /* Setup for the transfer */

  msg.frequency = CONFIG_CDC_I2C_FREQUENCY,
  msg.addr      = priv->addr;
  msg.flags     = 0;
  msg.buffer    = (FAR uint8_t *)buffer;  /* Override const */
  msg.length    = buflen;

  /* Then perform the transfer. */

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  return (ret >= 0) ? OK : ret;
}


/****************************************************************************
 * Name: cdc_i2c_read
 *
 * Description:
 *   Read from the I2C device.
 *
 ****************************************************************************/

static int cdc_i2c_read(FAR struct cdc_dev_s *priv,
                            FAR uint8_t *buffer, int buflen)
{
  struct i2c_msg_s msg;
  int ret;

  /* Setup for the transfer */

  msg.frequency = CONFIG_CDC_I2C_FREQUENCY,
  msg.addr      = priv->addr,
  msg.flags     = I2C_M_READ;
  msg.buffer    = buffer;
  msg.length    = buflen;

  /* Then perform the transfer. */

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: cdc_measure
 *
 * Description:
 *   Measure the raw temperature and the raw capacitance.
 *
 ****************************************************************************/

static int cdc_measure(FAR struct cdc_dev_s *priv)
{
  DEBUGASSERT(priv != NULL);

  if(priv == NULL) return -EINVAL;

  uint8_t	buf_raw[CDC_RAW_SAMPLE_SIZE];
  uint8_t	cdc_status;
  uint32_t	cdc_cap;
  uint32_t	cdc_temp;

  const int i2c_read = cdc_i2c_read(priv, buf_raw, CDC_RAW_SAMPLE_SIZE);

  if(i2c_read != OK) return i2c_read;

  /* separates cdc_measurements and the cdc_status
   * see datasheet AD7745 page 14 */

  cdc_status 	= buf_raw[0];							/* Status */
  cdc_cap		= (buf_raw[1] & 0xFFFFFFFF) << 8;		/* Cap Data H */
  cdc_cap		= (cdc_cap ^ buf_raw[2]) << 8;			/* Cap Data M */
  cdc_cap		= cdc_cap ^ buf_raw[3];					/* Cap Data L */
  cdc_temp		= (buf_raw[4] & 0xFFFFFFFF) << 8;		/* VT Data H */
  cdc_temp		= (cdc_temp ^ buf_raw[5]) << 8;			/* VT Data M */
  cdc_temp		= cdc_temp ^ buf_raw[6];				/* VT Data L */

  priv->status_cdc	= cdc_status;
  priv->cap			= cdc_cap;
  priv->temp		= cdc_temp;

  return OK;
}

/****************************************************************************
 * Name: cdc_init
 *
 * Description:
 *     Initializes a CDC device.
 *
 ****************************************************************************/

static int cdc_init(FAR struct cdc_dev_s *priv)
{
  DEBUGASSERT(priv != NULL);

  if(priv == NULL) return -EINVAL;

  uint8_t buf_i2c_tx[7] = {
      0x07,
      0x80,           // CAP Setup // Nur Bit 7 gesetzt
      0x80,           // VT Setup
      0x1B, //0x6B;   // EXC Setup
      0x01,           // configuration
      200,            // amp-factor
      0x00            // CapDac B
  };

  return cdc_i2c_write(priv, buf_i2c_tx, 7);
}

/****************************************************************************
 * Name: cdc_open
 *
 * Description:
 *   This method is called when the device is opened.
 *
 ****************************************************************************/

static int cdc_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: cdc_close
 *
 * Description:
 *   This method is called when the device is closed.
 *
 ****************************************************************************/

static int cdc_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: cdc_read
 *
 * Description:
 *   A dummy read method.
 *
 ****************************************************************************/

static ssize_t cdc_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  ssize_t size;
  FAR struct inode *inode     = filep->f_inode;
  FAR struct cdc_dev_s *priv  = inode->i_private;
  FAR struct cdc_measure_s *p = (FAR struct cdc_measure_s *)buffer;

  size = buflen;
  while(size >= sizeof(*p))
    {
      if (cdc_measure(priv) < 0)
        {
          return -1;
        }
      p->cdcStatus			= priv->status_cdc;
      p->cdcCapValue		= priv->cap;
      p->cdcVoltTempValue	= priv->temp;

      p++;
      size -= sizeof(*p);
    }

  return buflen;
}

/****************************************************************************
 * Name: cdc_write
 *
 * Description:
 *   A dummy write method.
 *
 ****************************************************************************/

static ssize_t cdc_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: cdc_ioctl
 *
 * Description:
 *   The standard ioctl method.
 *
 ****************************************************************************/

static int cdc_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct cdc_dev_s *priv  = inode->i_private;
  int                      ret   = OK;

  /* Handle ioctl commands */

  switch (cmd)
    {
      /* Measure the temperature and the pressure. Arg: None. */

      case SNIOC_MEASURE:
        DEBUGASSERT(arg == 0);
        ret = cdc_measure(priv);
        break;

      /* Unrecognized commands */

      default:
        snerr("ERROR: Unrecognized cmd: %d arg: %ld\n", cmd, arg);
        ret = -ENOTTY;
        break;
    }

  return ret;
}



/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cdc_register
 *
 * Description:
 *   Register the CDC character device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register, e.g., "/dev/press0".
 *   i2c     - An I2C driver instance.
 *   addr    - The I2C address of the CDC.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int cdc_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                    uint8_t addr)
{
  FAR struct cdc_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(i2c != NULL);

  /* Initialize the device's structure */

  priv = (FAR struct cdc_dev_s *)kmm_malloc(sizeof(*priv));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c   = i2c;
  priv->addr  = addr;

  /* Configure the device */

  ret = cdc_init(priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to configure device: %d\n", ret);
      goto errout;
    }


  /* Register the character driver */

  ret = register_driver(devpath, &g_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      goto errout;
    }

  return ret;

errout:
  kmm_free(priv);
  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_CDC */

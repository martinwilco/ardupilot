/****************************************************************************
 * drivers/sensors/tsys01.c
 * Character driver for MEAS TSYS01 Altimeters
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
#include <assert.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/tsys01.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_TSYS01)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_TSYS01_I2C_FREQUENCY
#  define CONFIG_TSYS01_I2C_FREQUENCY I2C_SPEED_FAST  /* 400 kHz */
#endif

#ifndef OK
#  define OK 0
#endif

/* Register Definitions *****************************************************/
/* Register Addresses */

#define TSYS01_READ_ADC_TEMP_REG   0x00    /* Read ADC Temperature result */
#define TSYS01_RESET_REG           0x1E    /* Reset Register */
#define TSYS01_START_ADC_CONV_REG  0x48    /* Start ADC Temperature Conversion */
#define TSYS01_PROM_ADDR0_REG      0xA0    /* PROM Read Address 0 Register */
#define TSYS01_PROM_ADDR1_REG      0xA2    /* PROM Read Address 1 (Coefficient c4) Register */
#define TSYS01_PROM_ADDR2_REG      0xA4    /* PROM Read Address 2 (Coefficient c3) Register */
#define TSYS01_PROM_ADDR3_REG      0xA6    /* PROM Read Address 3 (Coefficient c2) Register */
#define TSYS01_PROM_ADDR4_REG      0xA8    /* PROM Read Address 4 (Coefficient c1) Register */
#define TSYS01_PROM_ADDR5_REG      0xAA    /* PROM Read Address 5 (Coefficient c0) Register */
#define TSYS01_PROM_ADDR6_REG      0xAC    /* PROM Read Address 6 (SN 23...8 )     Register */
#define TSYS01_PROM_ADDR7_REG      0xAE    /* PROM Read Address 7 (SN.. and checksum) */

/* PROM Definitions *********************************************************/

#define TSYS01_PROM_LEN            8

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct tsys01_dev_s
{
  FAR struct i2c_master_s *i2c;     /* I2C interface */
  uint8_t               addr;       /* I2C address */
  uint32_t              SN;         /* 24 bit serialnumber */

  /* Measurement */

  float                 temp;

  /* Calibration coefficients */

  uint16_t              c0;
  uint16_t              c1;
  uint16_t              c2;
  uint16_t              c3;
  uint16_t              c4;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* CRC Calculation */

static uint8_t tsys01_checkCRC(FAR uint16_t *prom, uint8_t len);

/* I2C Helpers */

static int tsys01_i2c_write(FAR struct tsys01_dev_s *priv,
                             FAR const uint8_t *buffer, int buflen);
static int tsys01_i2c_read(FAR struct tsys01_dev_s *priv,
                            FAR uint8_t *buffer, int buflen);
static int tsys01_readu16(FAR struct tsys01_dev_s *priv, uint8_t regaddr,
                          FAR uint16_t *regval);
static int tsys01_readadc(FAR struct tsys01_dev_s *priv, FAR uint32_t *adc);
static int tsys01_reset(FAR struct tsys01_dev_s *priv);
static int tsys01_readprom(FAR struct tsys01_dev_s *priv);
static int tsys01_measure(FAR struct tsys01_dev_s *priv);
static float tsys01_convert(FAR struct tsys01_dev_s *priv, uint16_t raw_adc);

/* Character Driver Methods */

static int     tsys01_open(FAR struct file *filep);
static int     tsys01_close(FAR struct file *filep);
static ssize_t tsys01_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t tsys01_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static int     tsys01_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_fops =
{
  tsys01_open,
  tsys01_close,
  tsys01_read,
  tsys01_write,
  NULL,
  tsys01_ioctl,
  NULL
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tsys01_crc
 *
 * Description:
 *   Calculate the CRC.
 *
 ****************************************************************************/

static uint8_t tsys01_checkCRC(FAR uint16_t *src, uint8_t len)
{
    /* We need to interprete the PROM bytewise here */

    uint8_t* byte = (uint8_t*) src;

    uint16_t sum = 0x00;
    for(int i = 0; i < 2*len; i++)
    {
        sum += byte[i];
    }

    /* The lower half of the sum result has to be 0x00. */

    return ((0x00FF & sum) == 0x00) ? OK : -1;
}

/****************************************************************************
 * Name: tsys01_i2c_write
 *
 * Description:
 *   Write to the I2C device.
 *
 ****************************************************************************/

static int tsys01_i2c_write(FAR struct tsys01_dev_s *priv,
                             FAR const uint8_t *buffer, int buflen)
{
  struct i2c_msg_s msg;
  int ret;

  /* Setup for the transfer */

  msg.frequency = CONFIG_TSYS01_I2C_FREQUENCY,
  msg.addr      = priv->addr;
  msg.flags     = 0;
  msg.buffer    = (FAR uint8_t *)buffer;  /* Override const */
  msg.length    = buflen;

  /* Then perform the transfer. */

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: tsys01_i2c_read
 *
 * Description:
 *   Read from the I2C device.
 *
 ****************************************************************************/

static int tsys01_i2c_read(FAR struct tsys01_dev_s *priv,
                            FAR uint8_t *buffer, int buflen)
{
  struct i2c_msg_s msg;
  int ret;

  /* Setup for the transfer */

  msg.frequency = CONFIG_TSYS01_I2C_FREQUENCY,
  msg.addr      = priv->addr,
  msg.flags     = I2C_M_READ;
  msg.buffer    = buffer;
  msg.length    = buflen;

  /* Then perform the transfer. */

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: tsys01_readu16
 *
 * Description:
 *   Read from a 16-bit register.
 *
 ****************************************************************************/

static int tsys01_readu16(FAR struct tsys01_dev_s *priv, uint8_t regaddr,
                          FAR uint16_t *regval)
{
  uint8_t buffer[2];
  int ret;

  sninfo("addr: %02x\n", regaddr);

  /* Write the register address */

  ret = tsys01_i2c_write(priv, &regaddr, sizeof(regaddr));
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Restart and read 16 bits from the register */

  ret = tsys01_i2c_read(priv, buffer, sizeof(buffer));
  if (ret < 0)
    {
      snerr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  *regval = (uint16_t)buffer[0] << 8 | (uint16_t)buffer[1];
  sninfo("value: %04x ret: %d\n", *regval, ret);
  return ret;
}

/****************************************************************************
 * Name: tsys01_readadc
 *
 * Description:
 *   Read from the ADC register.
 *
 ****************************************************************************/

static int tsys01_readadc(FAR struct tsys01_dev_s *priv, FAR uint32_t *adc)
{
  uint8_t regaddr;
  uint8_t buffer[3];
  int ret;

  regaddr = TSYS01_READ_ADC_TEMP_REG;
  sninfo("addr: %02x\n", regaddr);

  /* Write the register address */

  ret = tsys01_i2c_write(priv, &regaddr, sizeof(regaddr));
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Restart and read 24 bits from the register */

  ret = tsys01_i2c_read(priv, buffer, sizeof(buffer));
  if (ret < 0)
    {
      snerr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  *adc = (uint32_t)buffer[0] << 16 |
         (uint32_t)buffer[1] << 8 |
         (uint32_t)buffer[2];

  sninfo("adc: %06lx ret: %d\n", *adc, ret);
  return ret;
}

/****************************************************************************
 * Name: tsys01_readprom
 *
 * Description:
 *   Starts the PROM read sequence. 
 *   It consists of three parts: 
 *          1. Set up the system to PROM read mode
 *          2. Get data from the PROM
 *          3. Check CRC
 *
 ****************************************************************************/

static int tsys01_readprom(FAR struct tsys01_dev_s *priv)
{
  int ret = OK;

  uint16_t prom[TSYS01_PROM_LEN];

  for (int i = 0; i < TSYS01_PROM_LEN; i++)
    {
      ret = tsys01_readu16(priv, TSYS01_PROM_ADDR0_REG + i * 2, &prom[i]);
      if (ret < 0)
        {
          snerr("ERROR: tsys01_readu16 failed: %d\n", ret);
          return ret;
        }
      usleep(100);
    }

  const int crc = tsys01_checkCRC(prom, TSYS01_PROM_LEN);

  if(crc != OK)
    {
      snerr("ERROR: crc mismatch\n");
      return -ENODEV;
    }

  priv->c4 = prom[1];
  priv->c3 = prom[2];
  priv->c2 = prom[3];
  priv->c1 = prom[4];
  priv->c0 = prom[5];

  priv->SN = ((((uint32_t) prom[6]) << 8) + (((uint32_t) prom[7]) >> 8));

  return ret;
}

/****************************************************************************
 * Name: tsys01_reset
 *
 * Description:
 *   Reset the device.
 *
 ****************************************************************************/

static int tsys01_reset(FAR struct tsys01_dev_s *priv)
{
  uint8_t regaddr;
  int ret;

  regaddr = TSYS01_RESET_REG;
  sninfo("addr: %02x\n", regaddr);

  /* Write the reset cmd */

  ret = tsys01_i2c_write(priv, &regaddr, sizeof(regaddr));
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Sensor needs some time between the commands.. */

  usleep(100);

  /* Check the CRC and read the calibration coefficients */

  ret = tsys01_readprom(priv);
  if (ret < 0)
    {
      snerr("ERROR: tsys01_readprom failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: tsys01_startConversion
 *
 * Description:
 *   Starts a conversion by sending the conversion command. 
 *   The sensor stays busy until conversion is done. When conversion is 
 *   finished the data can be accessed by using the ADC read function.
 *
 ****************************************************************************/

static int tsys01_startConversion(FAR struct tsys01_dev_s *priv)
{
  uint8_t regaddr;
  int ret;

  regaddr = TSYS01_START_ADC_CONV_REG;
  sninfo("addr: %02x\n", regaddr);

  /* Write the register address */

  ret = tsys01_i2c_write(priv, &regaddr, sizeof(regaddr));
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: tsys01_convert
 *
 * Description:
 *   Measure the uncompensated temperature or the uncompensated pressure.
 *
 ****************************************************************************/

static float tsys01_convert(FAR struct tsys01_dev_s *priv, uint16_t raw_adc) 
{
    /* pow() seems to be too slow, so we multiply manually */

  float retVal =
     (-2.0f) * priv->c4 * 1e-21 * raw_adc * raw_adc * raw_adc * raw_adc +
      (4.0f) * priv->c3 * 1e-16 * raw_adc * raw_adc * raw_adc           +
     (-2.0f) * priv->c2 * 1e-11 * raw_adc * raw_adc                     +
      (1.0f) * priv->c1 * 1e-6  * raw_adc                               +
     (-1.5f) * priv->c0 * 1e-2;

  return retVal;
}

/****************************************************************************
 * Name: tsys01_measure
 *
 * Description:
 *   Measure the temperature.
 *
 ****************************************************************************/

static int tsys01_measure(FAR struct tsys01_dev_s *priv)
{
  int ret;

  /* Read the value from the ADC */

  uint32_t adc24;     /* 24 bit raw temp value */

  ret = tsys01_readadc(priv, &adc24);
  if (ret < 0)
    {
      snerr("ERROR: tsys01_readadc failed: %d\n", ret);
      return ret;
    }

  const uint16_t adc16 = adc24 / 256;

  /* Convert the raw value to a valid floating degree value */

  priv->temp = tsys01_convert(priv, adc16);

  return OK;
}

/****************************************************************************
 * Name: tsys01_open
 *
 * Description:
 *   This method is called when the device is opened.
 *
 ****************************************************************************/

static int tsys01_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: tsys01_close
 *
 * Description:
 *   This method is called when the device is closed.
 *
 ****************************************************************************/

static int tsys01_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: tsys01_read
 *
 * Description:
 *   A dummy read method.
 *
 ****************************************************************************/

static ssize_t tsys01_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct inode *inode        = filep->f_inode;
  FAR struct tsys01_dev_s *priv  = inode->i_private;
  FAR struct tsys01_measure_s *p = (FAR struct tsys01_measure_s *)buffer;

  if (buflen >= sizeof(*p))
    {
      if (tsys01_measure(priv) < 0)
        {
          return -1;
        }

      p->temperature  = priv->temp;
    }

  return buflen;
}

/****************************************************************************
 * Name: tsys01_write
 *
 * Description:
 *   A dummy write method.
 *
 ****************************************************************************/

static ssize_t tsys01_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: tsys01_ioctl
 *
 * Description:
 *   The standard ioctl method.
 *
 ****************************************************************************/

static int tsys01_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct tsys01_dev_s *priv  = inode->i_private;
  int                      ret   = OK;

  /* Handle ioctl commands */

  switch (cmd)
    {
      /* Starts the conversion sequence. Arg: None. */

      case SNIOC_START_CONVERSION:
        ret = tsys01_startConversion(priv);
        break;

      /* Measure the temperature and the pressure. Arg: None. */

      case SNIOC_MEASURE:
        ret = tsys01_measure(priv);
        break;

      /* Return the temperature last measured. Arg: float* pointer. */

      case SNIOC_TEMPERATURE:
        {
          FAR float *ptr = (FAR float *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);
          *ptr = priv->temp;
          sninfo("temp: %3.3f\n", *ptr);
        }
        break;

      /* Reset the device. Arg: None. */

      case SNIOC_RESET:
        ret = tsys01_reset(priv);
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
 * Name: tsys01_register
 *
 * Description:
 *   Register the TSYS01 character device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register, e.g., "/dev/press0".
 *   i2c     - An I2C driver instance.
 *   addr    - The I2C address of the TSYS01.
 *   osr     - The oversampling ratio.
 *   model   - The TSYS01 model.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int tsys01_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                    uint8_t addr)
{
  FAR struct tsys01_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(i2c != NULL);
  DEBUGASSERT(addr == TSYS01_ADDR);

  /* Initialize the device's structure */

  priv = (FAR struct tsys01_dev_s *)kmm_malloc(sizeof(*priv));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c   = i2c;
  priv->addr  = addr;
  priv->SN    = 0;

  priv->temp  = 0;

  priv->c0    = 0;
  priv->c1    = 0;
  priv->c2    = 0;
  priv->c3    = 0;
  priv->c4    = 0;

  ret = tsys01_reset(priv);
  if (ret < 0)
    {
      snerr("ERROR: tsys01_reset failed: %d\n", ret);
      goto errout;
    }

  /* Trigger already initial measurement */

  ret = tsys01_startConversion(priv);
  if (ret < 0)
    {
      snerr("ERROR: tsys01_startConversion failed: %d\n", ret);
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

#endif /* CONFIG_I2C && CONFIG_SENSORS_TSYS01 */

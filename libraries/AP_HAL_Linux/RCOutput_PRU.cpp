#include "RCOutput_PRU.h"

#include <dirent.h>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <AP_HAL/AP_HAL.h>

using namespace Linux;

#define PWM_CHAN_COUNT 12

#define DUMP_CH_EN 0

static const uint8_t chan_pru_map[]= {10,8,11,9,7,6,5,4,3,2,1,0};                //chan_pru_map[CHANNEL_NUM] = PRU_REG_R30/31_NUM;
static const uint8_t pru_chan_map[]= {11,10,9,8,7,6,5,4,1,3,0,2};                //pru_chan_map[PRU_REG_R30/31_NUM] = CHANNEL_NUM;

static void catch_sigbus(int sig)
{
    AP_HAL::panic("RCOutput.cpp:SIGBUS error gernerated\n");
}
void RCOutput_PRU::init()
{
    uint32_t mem_fd;
    signal(SIGBUS,catch_sigbus);
    mem_fd = open("/dev/mem", O_RDWR|O_SYNC|O_CLOEXEC);
    sharedMem_cmd = (struct pwm_cmd *) mmap(0, 0x1000, PROT_READ|PROT_WRITE,
                                            MAP_SHARED, mem_fd, RCOUT_PRUSS_SHAREDRAM_BASE);
    if(MAP_FAILED == sharedMem_cmd)
    {
        AP_HAL::panic("Failed to mmap PRU1 SHM\n");
    }
    memset((void *)sharedMem_cmd, 0x0, 0x1000);
    close(mem_fd);

    // all outputs default to 50Hz, the top level vehicle code
    // overrides this when necessary
    set_freq(0xFFFFFFFF, 50);
}

void RCOutput_PRU::set_freq(uint32_t chmask, uint16_t freq_hz)            //LSB corresponds to CHAN_1
{
    uint8_t i;
    unsigned long tick=TICK_PER_S/(unsigned long)freq_hz;

    for (i=0;i<PWM_CHAN_COUNT;i++) {
        if (chmask & (1U<<i)) {
            sharedMem_cmd->periodhi[chan_pru_map[i]][0]=tick;
        }
    }
    set_magic_sync();
}

uint16_t RCOutput_PRU::get_freq(uint8_t ch)
{
    if(sharedMem_cmd->hilo_read[chan_pru_map[ch]][0])
    {
        return TICK_PER_S/sharedMem_cmd->hilo_read[chan_pru_map[ch]][0];
    }
    else
    {
        // hal.util->prt("[Warn] PRU still not write back freq");
        return 0;
    }
}

void RCOutput_PRU::enable_ch(uint8_t ch)
{
    sharedMem_cmd->enmask |= 1U<<chan_pru_map[ch];
}

void RCOutput_PRU::disable_ch(uint8_t ch)
{
    sharedMem_cmd->enmask &= ~(1U<<chan_pru_map[ch]);
    set_magic_sync();
}

void RCOutput_PRU::write(uint8_t ch, uint16_t period_us)
{
    if (corked) {
        pending[ch] = period_us;
        pending_mask |= (1U << ch);
    } else {
        sharedMem_cmd->periodhi[chan_pru_map[ch]][1] = TICK_PER_US*period_us;
    }
}

uint16_t RCOutput_PRU::read(uint8_t ch)
{
    return (sharedMem_cmd->hilo_read[chan_pru_map[ch]][1]/TICK_PER_US);
}

void RCOutput_PRU::read(uint16_t* period_us, uint8_t len)
{
    uint8_t i;
    if(len>PWM_CHAN_COUNT){
        len = PWM_CHAN_COUNT;
    }
    for(i=0;i<len;i++){
        period_us[i] = sharedMem_cmd->hilo_read[chan_pru_map[i]][1]/TICK_PER_US;
    }
}

void RCOutput_PRU::cork(void)
{
    corked = true;
}

void RCOutput_PRU::push(void)
{
    corked = false;
    for (uint8_t i=0; i<ARRAY_SIZE(pending); i++) {
        if (pending_mask & (1U << i)) {
            write(i, pending[i]);
        }
    }
    set_magic_sync();
    pending_mask = 0;
}


// will be invoked 1KHz, meanwhile PRU will check alive 1Hz
void RCOutput_PRU::rcout_keep_alive(void)
{
    static uint32_t delay_cnt = 0;
    if(++delay_cnt < 20)
    {
        return;
    }
    delay_cnt = 0; 
#if 0
    static uint32_t cnt1 = 0;
    if((0 == cnt1%100) || (1 == cnt1%100))
    {
        printf("[%d us] RCOut Keep alive\n", AP_HAL::micros()); 
    }
    cnt1++;
#endif
#if DUMP_CH_EN 
    static uint32_t cnt  = 0;
    if(!(++cnt%80))
    {
        printf("CH_EN: 0x%04x\n", sharedMem_cmd->enmask);
        printf("CH9: duty: %d us, period: %d us\n", read(8), get_freq(8)); // sharedMem_cmd->enmask);
        printf("CH11: duty: %d us, period: %d us\n", read(10), get_freq(10)); // sharedMem_cmd->enmask);
        printf("CH10: duty: %d us, period: %d us\n", read(9), get_freq(9)); // sharedMem_cmd->enmask);
        printf("CH4: duty: %d us, period: %d us\n", read(3), get_freq(3)); // sharedMem_cmd->enmask);
        printf("CH5: duty: %d us, period: %d us\n", read(4), get_freq(4)); // sharedMem_cmd->enmask);
        printf("CH6: duty: %d us, period: %d us\n", read(5), get_freq(5)); // sharedMem_cmd->enmask);
    }
#endif
#ifdef KEEP_ALIVE_WITH_PRU
    static unsigned int time_out = 0;
    static unsigned int wait_pru_time = 0;
    // check keep alive with PRU (first time: config timeout to PRU)
    if(time_out > 1)
    {
        // reply alive
        if(PWM_REPLY_KEEP_ALIVE == (sharedMem_cmd->keep_alive & 0xFFFF))
        {
            // cmd alive
            sharedMem_cmd->keep_alive = PWM_CMD_KEEP_ALIVE; 
            time_out = 2;
        }
        else if(PWM_CMD_KEEP_ALIVE == (sharedMem_cmd->keep_alive & 0xFFFF))
        {
            time_out++;
            // PRU should be dead
            if(time_out > (KEEP_ALIVE_TIME_OUT_HOST*50))
            {
                ::printf("Warning: PRU didn't reply for more than %d seconds (50Hz: %d), should be dead!\n", KEEP_ALIVE_TIME_OUT_HOST, time_out);
                time_out = 2;
            }
        }
        else
        {
            ::printf("Error: unknown PRU keep alive code 0x%04x(!= 0x%04x) !\n", sharedMem_cmd->keep_alive & 0xFFFF, PWM_REPLY_KEEP_ALIVE);
        }
    }
    else if(1 == time_out) // wait for 1st PRU reply (PRU wake up)
    {
        // reply alive
        if(PWM_REPLY_KEEP_ALIVE == (sharedMem_cmd->keep_alive & 0xFFFF))
        {
            // cmd alive
            sharedMem_cmd->keep_alive = PWM_CMD_KEEP_ALIVE; 
            time_out = 2;
            ::printf("[OK]: PRU alive\n");
        }
        else if(PWM_CMD_KEEP_ALIVE == (sharedMem_cmd->keep_alive & 0xFFFF))
        {
            sharedMem_cmd->time_out = KEEP_ALIVE_TIME_OUT_PRU; 
            wait_pru_time++; 
            if(wait_pru_time > (PRU_POWER_UP_TIME*50))
            {
                wait_pru_time = 0;
                ::printf("Warning: PRU still not wakeup...\n");
            }
        }
        else
        {
            ::printf("Warning: unknown PRU keep alive code!\n");
        }
    }
    else // time_out == 0
    {
        sharedMem_cmd->time_out = KEEP_ALIVE_TIME_OUT_PRU; 
        sharedMem_cmd->keep_alive = PWM_CMD_KEEP_ALIVE; 
        time_out = 1;
    }
#endif
}

void RCOutput_PRU::set_magic_sync(void)
{
#if 0
	static char first_time = 1;
	if (first_time == 1)
	{
       sharedMem_cmd->magic = PWM_CMD_MAGIC;
       first_time = 0;
	}
	else
	{
		if (sharedMem_cmd->magic != PWM_REPLY_MAGIC )
			printf("11111\n");
	}
#endif
    sharedMem_cmd->magic = PWM_CMD_MAGIC;
}

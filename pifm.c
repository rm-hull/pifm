// To run:
//  g++ -O3 -o pifm pifm.c
//  ./pifm left_right.wav 103.3 22050 stereo
//  ./pifm sound.wav

// Created by Oliver Mattos and Oskar Weigl.
// Code quality = Totally hacked together.


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <dirent.h>
#include <math.h>
#include <fcntl.h>
#include <assert.h>
#include <malloc.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <signal.h>
#include <unistd.h>

#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)

#define PI 3.14159265

int  mem_fd;
char *gpio_mem, *gpio_map;
char *spi0_mem, *spi0_map;


// I/O access
volatile unsigned *gpio;
volatile unsigned *allof7e;

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0
#define GPIO_GET *(gpio+13)  // sets   bits which are 1 ignores bits which are 0

#define ACCESS(base) *(volatile int*)((int)allof7e+base-0x7e000000)
#define SETBIT(base, bit) ACCESS(base) |= 1<<bit
#define CLRBIT(base, bit) ACCESS(base) &= ~(1<<bit)

#define CM_GP0CTL (0x7e101070)
#define GPFSEL0 (0x7E200000)
#define CM_GP0DIV (0x7e101074)
#define CLKBASE (0x7E101000)
#define DMABASE (0x7E007000)
#define PWMBASE  (0x7e20C000) /* PWM controller */


struct GPCTL {
    char SRC         : 4;
    char ENAB        : 1;
    char KILL        : 1;
    char             : 1;
    char BUSY        : 1;
    char FLIP        : 1;
    char MASH        : 2;
    unsigned int     : 13;
    char PASSWD      : 8;
};



void getRealMemPage(void** vAddr, void** pAddr) {
    void* a = valloc(4096);
    
    ((int*)a)[0] = 1;  // use page to force allocation.
    
    mlock(a, 4096);  // lock into ram.
    
    *vAddr = a;  // yay - we know the virtual address
    
    unsigned long long frameinfo;
    
    int fp = open("/proc/self/pagemap", 'r');
    lseek(fp, ((int)a)/4096*8, SEEK_SET);
    read(fp, &frameinfo, sizeof(frameinfo));
    
    *pAddr = (void*)((int)(frameinfo*4096));
}

void freeRealMemPage(void* vAddr) {
    
    munlock(vAddr, 4096);  // unlock ram.
    
    free(vAddr);
}

void setup_fm()
{

    /* open /dev/mem */
    if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
        printf("can't open /dev/mem \n");
        exit (-1);
    }
    
    allof7e = (unsigned *)mmap(
                  NULL,
                  0x01000000,  //len
                  PROT_READ|PROT_WRITE,
                  MAP_SHARED,
                  mem_fd,
                  0x20000000  //base
              );

    if ((int)allof7e==-1) exit(-1);

    SETBIT(GPFSEL0 , 14);
    CLRBIT(GPFSEL0 , 13);
    CLRBIT(GPFSEL0 , 12);

 
    struct GPCTL setupword = {6/*SRC*/, 1, 0, 0, 0, 1,0x5a};

    ACCESS(CM_GP0CTL) = *((int*)&setupword);
}


void modulate(int m)
{
    ACCESS(CM_GP0DIV) = (0x5a << 24) + 0x4d72 + m;
}

struct CB {
    volatile unsigned int TI;
    volatile unsigned int SOURCE_AD;
    volatile unsigned int DEST_AD;
    volatile unsigned int TXFR_LEN;
    volatile unsigned int STRIDE;
    volatile unsigned int NEXTCONBK;
    volatile unsigned int RES1;
    volatile unsigned int RES2;
    
};

struct DMAregs {
    volatile unsigned int CS;
    volatile unsigned int CONBLK_AD;
    volatile unsigned int TI;
    volatile unsigned int SOURCE_AD;
    volatile unsigned int DEST_AD;
    volatile unsigned int TXFR_LEN;
    volatile unsigned int STRIDE;
    volatile unsigned int NEXTCONBK;
    volatile unsigned int DEBUG;
};

struct PageInfo {
    void* p;  // physical address
    void* v;   // virtual address
};

struct PageInfo constPage;   
struct PageInfo instrPage;
#define BUFFERINSTRUCTIONS 65536
struct PageInfo instrs[BUFFERINSTRUCTIONS];



class SampleSink{
public:
    virtual void consume(float* data, int dataLen){};  // floating point samples
    virtual void consume(void* data, int dataLen){};  // raw data, len in bytes.
};

class Outputter : public SampleSink {
public:
    int bufPtr;
    float clocksPerSample;
    const int sleeptime;
    float fracerror;
    float timeErr;
    
    Outputter(float rate):
        sleeptime((float)1e6 * BUFFERINSTRUCTIONS/4/rate/2),   // sleep time is half of the time to empty the buffer
        fracerror(0),
        timeErr(0) {
        clocksPerSample = 22500.0 / rate * 1373.5;  // for timing, determined by experiment
        bufPtr=0;
    };
    void consume(float* data, int num) {
        for (int i=0; i<num; i++) {
            float value = data[i]*8;  // modulation index (AKA volume!)
           
            // dump raw baseband data to stdout for audacity analysis.
            //write(1, &value, 4);
            
            // debug code.  Replaces data with a set of tones.
            //static int debugCount;
            //debugCount++;
            //value = (debugCount & 0x1000)?0.5:0;  // two different tests
            //value += 0.2 * ((debugCount & 0x8)?1.0:-1.0);   // tone
            //if (debugCount & 0x2000) value = 0;   // silence 
            // end debug code
            
            value += fracerror;  // error that couldn't be encoded from last time.
            
            int intval = (int)(round(value));  // integer component
            float frac = (value - (float)intval + 1)/2;
            unsigned int fracval = round(frac*clocksPerSample); // the fractional component
            
            // we also record time error so that if one sample is output
            // for slightly too long, the next sample will be shorter.
            timeErr = timeErr - int(timeErr) + clocksPerSample;
            
            fracerror = (frac - (float)fracval*(1.0-2.3/clocksPerSample)/clocksPerSample)*2;  // error to feed back for delta sigma
            
            // Note, the 2.3 constant is because our PWM isn't perfect.
            // There is a finite time for the DMA controller to load a new value from memory,
            // Therefore the width of each pulse we try to insert has a constant added to it.
            // That constant is about 2.3 bytes written to the serializer, or about 18 cycles.  We use delta sigma
            // to correct for this error and the pwm timing quantization error.
            
            // To reduce noise, rather than just rounding to the nearest clock we can use, we PWM between
            // the two nearest values.
            
            // delay if necessary.  We can also print debug stuff here while not breaking timing.
            static int time;
            time++;
                
            while( (ACCESS(DMABASE + 0x04 /* CurBlock*/) & ~ 0x7F) ==  (int)(instrs[bufPtr].p)) {
                usleep(sleeptime);  // are we anywhere in the next 4 instructions?
            }
            
            // Create DMA command to set clock controller to output FM signal for PWM "LOW" time.
            ((struct CB*)(instrs[bufPtr].v))->SOURCE_AD = (int)constPage.p + 2048 + intval*4 - 4 ;
            bufPtr++;
            
            // Create DMA command to delay using serializer module for suitable time.
            ((struct CB*)(instrs[bufPtr].v))->TXFR_LEN = (int)timeErr-fracval;
            bufPtr++;
            
            // Create DMA command to set clock controller to output FM signal for PWM "HIGH" time.
            ((struct CB*)(instrs[bufPtr].v))->SOURCE_AD = (int)constPage.p + 2048 + intval*4 + 4;
            bufPtr++;
            
            // Create DMA command for more delay.
            ((struct CB*)(instrs[bufPtr].v))->TXFR_LEN = fracval;
            bufPtr=(bufPtr+1) % (BUFFERINSTRUCTIONS);
        }
    }
};

class PreEmp : public SampleSink {
public:
    float fmconstant;
    float dataold;
    SampleSink* next;
    
    // this isn't the right filter...  But it's close...
    // Something todo with a bilinear transform not being right...
    PreEmp(float rate, SampleSink* next): 
        fmconstant(rate * 75.0e-6), // for pre-emphisis filter.  75us time constant
        dataold(0),
        next(next) { };
    
    
    void consume(float* data, int num) {
        for (int i=0; i<num; i++) {
            float value = data[i];
            
            float sample = value + (dataold-value) / (1-fmconstant);  // fir of 1 + s tau
            
            next->consume(&sample, 1);
            
            dataold = value;
        }
    }
};


class Resamp : public SampleSink {
public:
    static const int QUALITY = 5;    // comp. complexity goes up linearly with this.
    static const int SQUALITY = 10;  // start time quality (defines max phase error of filter vs ram used & cache thrashing)
    static const int BUFSIZE = 1000;
    float dataOld[QUALITY];
    float sincLUT[SQUALITY][QUALITY]; // [startime][samplenum]
    float ratio;
    float outTimeLeft;
    float outBuffer[BUFSIZE];
    int outBufPtr;
    SampleSink* next;
    
    Resamp(float rateIn, float rateOut, SampleSink* next): 
        outTimeLeft(1.0),
        outBufPtr(0),
        ratio((float)rateIn/(float)rateOut),
        next(next) {
        
        for(int i=0; i<QUALITY; i++) {  // sample
          for(int j=0; j<SQUALITY; j++) {  // starttime
            float x = PI * ((float)j/SQUALITY + (QUALITY-1-i) - (QUALITY-1)/2.0);
            if (x==0)
              sincLUT[j][i] = 1.0;  // sin(0)/(0) == 1, says my limits therory
            else
              sincLUT[j][i] = sin(x)/x;            
          }
        }
        
    };
    
    
    void consume(float* data, int num) {
        for (int i=0; i<num; i++) {
            // shift old data along
            for (int j=0; j<QUALITY-1; j++) {
              dataOld[j] = dataOld[j+1];
            }
            
            // put in new sample
            dataOld[QUALITY-1] = data[i];
            outTimeLeft -= 1.0;
            
            // go output this stuff!
            while (outTimeLeft<1.0) {
                float outSample = 0;
                int lutNum = (int)(outTimeLeft*SQUALITY);
                for (int j=0; j<QUALITY; j++) {
                    outSample += dataOld[j] * sincLUT[lutNum][j];
                }
                outBuffer[outBufPtr++] = outSample;
                outTimeLeft += ratio;
                
                // if we have lots of data, shunt it to the next stage.
                if (outBufPtr >= BUFSIZE) {
                  next->consume(outBuffer, outBufPtr);
                  outBufPtr = 0;
                }
            }
        }
    }
};

class NullSink: public SampleSink {
public:
    
    NullSink() { }
    
    void consume(float* data, int num) {}   // throws away data
};

// decodes a mono wav file
class Mono: public SampleSink {
public:
    SampleSink* next;
    
    Mono(SampleSink* next): next(next) { }
    
    void consume(void* data, int num) {    // expects num%2 == 0
        for (int i=0; i<num/2; i++) {
            float l = (float)(((short*)data)[i]) / 32768.0;
            next->consume( &l, 1);
        }
    }
};

class StereoSplitter: public SampleSink {
public:
    SampleSink* nextLeft;
    SampleSink* nextRight;
    
    StereoSplitter(SampleSink* nextLeft, SampleSink* nextRight): 
        nextLeft(nextLeft), nextRight(nextRight) { }
    
    
    void consume(void* data, int num) {    // expects num%4 == 0
        for (int i=0; i<num/2; i+=2) {
            float l = (float)(((short*)data)[i]) / 32768.0;
            nextLeft->consume( &l, 1);
            
            float r = (float)(((short*)data)[i+1]) / 32768.0;
            nextRight->consume( &r, 1);
        }
    }
};


const unsigned char RDSDATA[] = {
// RDS data.  Send MSB first.  Google search gr_rds_data_encoder.cc to make your own data.
    0x50, 0xFF, 0xA9, 0x01, 0x02, 0x1E, 0xB0, 0x00, 0x05, 0xA1, 0x41, 0xA4, 0x12, 
    0x50, 0xFF, 0xA9, 0x01, 0x02, 0x45, 0x20, 0x00, 0x05, 0xA1, 0x19, 0xB6, 0x8C, 
    0x50, 0xFF, 0xA9, 0x01, 0x02, 0xA9, 0x90, 0x00, 0x05, 0xA0, 0x80, 0x80, 0xDC, 
    0x50, 0xFF, 0xA9, 0x01, 0x03, 0xC7, 0xD0, 0x00, 0x05, 0xA0, 0x80, 0x80, 0xDC, 
    0x50, 0xFF, 0xA9, 0x09, 0x00, 0x14, 0x75, 0x47, 0x51, 0x7D, 0xB9, 0x95, 0x79, 
    0x50, 0xFF, 0xA9, 0x09, 0x00, 0x4F, 0xE7, 0x32, 0x02, 0x21, 0x99, 0xC8, 0x09, 
    0x50, 0xFF, 0xA9, 0x09, 0x00, 0xA3, 0x56, 0xF6, 0xD9, 0xE8, 0x81, 0xE5, 0xEE, 
    0x50, 0xFF, 0xA9, 0x09, 0x00, 0xF8, 0xC6, 0xF7, 0x5B, 0x19, 0xC8, 0x80, 0x88, 
    0x50, 0xFF, 0xA9, 0x09, 0x01, 0x21, 0xA5, 0x26, 0x19, 0xD5, 0xCD, 0xC3, 0xDC, 
    0x50, 0xFF, 0xA9, 0x09, 0x01, 0x7A, 0x36, 0x26, 0x56, 0x31, 0xC9, 0xC8, 0x72, 
    0x50, 0xFF, 0xA9, 0x09, 0x01, 0x96, 0x87, 0x92, 0x09, 0xA5, 0x41, 0xA4, 0x12, 
    0x50, 0xFF, 0xA9, 0x09, 0x01, 0xCD, 0x12, 0x02, 0x8C, 0x0D, 0xBD, 0xB6, 0xA6, 
    0x50, 0xFF, 0xA9, 0x09, 0x02, 0x24, 0x46, 0x17, 0x4B, 0xB9, 0xD1, 0xBC, 0xE2, 
    0x50, 0xFF, 0xA9, 0x09, 0x02, 0x7F, 0xD7, 0x34, 0x09, 0xE1, 0x9D, 0xB5, 0xFF, 
    0x50, 0xFF, 0xA9, 0x09, 0x02, 0x93, 0x66, 0x16, 0x92, 0xD9, 0xB0, 0xB9, 0x3E, 
    0x50, 0xFF, 0xA9, 0x09, 0x02, 0xC8, 0xF6, 0x36, 0xF4, 0x85, 0xB4, 0xA4, 0x74, 
    0x50, 0xFF, 0xA9, 0x09, 0x03, 0x11, 0x92, 0x02, 0x00, 0x00, 0x80, 0x80, 0xDC, 
    0x50, 0xFF, 0xA9, 0x09, 0x03, 0x4A, 0x02, 0x02, 0x00, 0x00, 0x80, 0x80, 0xDC, 
    0x50, 0xFF, 0xA9, 0x09, 0x03, 0xA6, 0xB2, 0x02, 0x00, 0x00, 0x80, 0x80, 0xDC, 
    0x50, 0xFF, 0xA9, 0x09, 0x03, 0xFD, 0x22, 0x02, 0x00, 0x00, 0x80, 0x80, 0xDC
};

class RDSEncoder: public SampleSink {
public:
    float sinLut[8];
    SampleSink* next;
    int bitNum;
    int lastBit;
    int time;
    float lastValue;
    
    RDSEncoder(SampleSink* next): 
        next(next), bitNum(0), lastBit(0), time(0), lastValue(0) {
        for (int i=0; i<8; i++) {
            sinLut[i] = sin((float)i*2.0*PI*3/8);
        }
    }
    
    void consume(float* data, int num) {
        for (int i=0; i<num; i++) {
            if (!time) {
              // time for a new bit
              int newBit = (RDSDATA[bitNum/8]>>(7-(bitNum%8)))&1;
              lastBit = lastBit^newBit;  // differential encoding
              
              bitNum = (bitNum+1)%(20*13*8);
            }
            
            int outputBit = (time<192)?lastBit:1-lastBit; // manchester encoding
            
            lastValue = lastValue*0.99 + (((float)outputBit)*2-1)*0.01;  // very simple IIR filter to hopefully reduce sidebands.
            data[i] += lastValue*sinLut[time%8]*0.05;
            
            time = (time+1)%384;
        }
        next->consume(data, num);
    }
};

// Takes 2 input signals at 152kHz and stereo modulates it.
class StereoModulator: public SampleSink {
public:

    // Helper to make two input interfaces for the stereomodulator.   Feels like I'm reimplementing a closure here... :-(
    class ModulatorInput: public SampleSink {
    public:
        StereoModulator* mod;
        int channel;
        
        ModulatorInput(StereoModulator* mod, int channel): 
            mod(mod),
            channel(channel) { }
        
        void consume(float* data, int num) {
            mod->consume(data, num, channel);
        }
    };
    
    float buffer[1024];
    int bufferOwner;
    int bufferLen;
    int state; // 8 state state machine.
    float sinLut[16];
    
    SampleSink* next;
    
    StereoModulator(SampleSink* next): 
        next(next), bufferOwner(0), bufferLen(0), state(0) {
        for (int i=0; i<16; i++) {
            sinLut[i] = sin((float)i*2.0*PI/8);
        }
    }
        
    SampleSink* getChannel(int channel) {
        return new ModulatorInput(this, channel);  // never freed, cos I'm a rebel...
    }    
    
    void consume(float* data, int num, int channel) {
        if (channel==bufferOwner || bufferLen==0) {
            bufferOwner=channel;
            // append to buffer
            while(num && bufferLen<1024) {
                buffer[bufferLen++] = data[0];
                data++;
                num--;
            }
        } else {
            int consumable = (bufferLen<num)?bufferLen:num;
            float* left = (bufferOwner==0)?buffer:data;
            float* right = (bufferOwner==1)?buffer:data;
            for (int i=0; i<consumable; i++) {
                state = (state+1) %8;
                // equation straight from wikipedia...
                buffer[i] = ((left[i]+right[i])/2 + (left[i]-right[i])/2*sinLut[state*2])*0.9 + 0.1*sinLut[state]; 
            }
            next->consume(buffer, consumable);
            
            // move stuff along buffer
            for (int i=consumable; i<bufferLen; i++) {
              buffer[i-consumable] = buffer[i];
            }
            bufferLen-=consumable;
            
            //reconsume any remaining data
            data += consumable;
            num -= consumable;
            consume(data, num, channel);
        }
    }
};


void playWav(char* filename, float samplerate, bool stereo)
{
    int fp= STDIN_FILENO;
    if(filename[0]!='-') fp = open(filename, 'r');
    
    char data[1024];
    
    SampleSink* ss;
    
    if (stereo) {
      StereoModulator* sm = new StereoModulator(new RDSEncoder(new Outputter(152000)));
      ss = new StereoSplitter( 
        // left
        new PreEmp(samplerate, new Resamp(samplerate, 152000, sm->getChannel(0))), 
        
        // Right
        new PreEmp(samplerate, new Resamp(samplerate, 152000, sm->getChannel(1)))
      );
    } else {
      ss = new Mono(new PreEmp(samplerate, new Outputter(samplerate)));
    }
    
    for (int i=0; i<22; i++)
       read(fp, &data, 2);  // read past header
    
    int readBytes;
    while (readBytes = read(fp, &data, 1024)) {
        
        ss->consume(data, readBytes);
    }
    close(fp);
}

void unSetupDMA(){
    printf("exiting\n");
    struct DMAregs* DMA0 = (struct DMAregs*)&(ACCESS(DMABASE));
    DMA0->CS =1<<31;  // reset dma controller
    
}

void handSig(int dunno) {
    exit(0);
}
void setupDMA( float centerFreq ){

  atexit(unSetupDMA);
  signal (SIGINT, handSig);
  signal (SIGTERM, handSig);
  signal (SIGHUP, handSig);
  signal (SIGQUIT, handSig);

   // allocate a few pages of ram
   getRealMemPage(&constPage.v, &constPage.p);
   
   int centerFreqDivider = (int)((500.0 / centerFreq) * (float)(1<<12) + 0.5);
   
   // make data page contents - it's essientially 1024 different commands for the
   // DMA controller to send to the clock module at the correct time.
   for (int i=0; i<1024; i++)
     ((int*)(constPage.v))[i] = (0x5a << 24) + centerFreqDivider - 512 + i;
   
   
   int instrCnt = 0;
   
   while (instrCnt<BUFFERINSTRUCTIONS) {
     getRealMemPage(&instrPage.v, &instrPage.p);
     
     // make copy instructions
     struct CB* instr0= (struct CB*)instrPage.v;
     
     for (int i=0; i<4096/sizeof(struct CB); i++) {
       instrs[instrCnt].v = (void*)((int)instrPage.v + sizeof(struct CB)*i);
       instrs[instrCnt].p = (void*)((int)instrPage.p + sizeof(struct CB)*i);
       instr0->SOURCE_AD = (unsigned int)constPage.p+2048;
       instr0->DEST_AD = PWMBASE+0x18 /* FIF1 */;
       instr0->TXFR_LEN = 4;
       instr0->STRIDE = 0;
       //instr0->NEXTCONBK = (int)instrPage.p + sizeof(struct CB)*(i+1);
       instr0->TI = (1/* DREQ  */<<6) | (5 /* PWM */<<16) |  (1<<26/* no wide*/) ;
       instr0->RES1 = 0;
       instr0->RES2 = 0;
       
       if (!(i%2)) {
         instr0->DEST_AD = CM_GP0DIV;
         instr0->STRIDE = 4;
         instr0->TI = (1<<26/* no wide*/) ;
       }
       
       if (instrCnt!=0) ((struct CB*)(instrs[instrCnt-1].v))->NEXTCONBK = (int)instrs[instrCnt].p;
       instr0++;
       instrCnt++;
     }
   }
   ((struct CB*)(instrs[BUFFERINSTRUCTIONS-1].v))->NEXTCONBK = (int)instrs[0].p;
   
   // set up a clock for the PWM
   ACCESS(CLKBASE + 40*4 /*PWMCLK_CNTL*/) = 0x5A000026;
   usleep(1000);
   ACCESS(CLKBASE + 41*4 /*PWMCLK_DIV*/)  = 0x5A002800;
   ACCESS(CLKBASE + 40*4 /*PWMCLK_CNTL*/) = 0x5A000016;
   usleep(1000); 

   // set up pwm
   ACCESS(PWMBASE + 0x0 /* CTRL*/) = 0;
   usleep(1000);
   ACCESS(PWMBASE + 0x4 /* status*/) = -1;  // clear errors
   usleep(1000);
   ACCESS(PWMBASE + 0x0 /* CTRL*/) = -1; //(1<<13 /* Use fifo */) | (1<<10 /* repeat */) | (1<<9 /* serializer */) | (1<<8 /* enable ch */) ;
   usleep(1000);
   ACCESS(PWMBASE + 0x8 /* DMAC*/) = (1<<31 /* DMA enable */) | 0x0707;
   
   //activate dma
   struct DMAregs* DMA0 = (struct DMAregs*)&(ACCESS(DMABASE));
   DMA0->CS =1<<31;  // reset
   DMA0->CONBLK_AD=0; 
   DMA0->TI=0; 
   DMA0->CONBLK_AD = (unsigned int)(instrPage.p);
   DMA0->CS =(1<<0)|(255 <<16);  // enable bit = 0, clear end flag = 1, prio=19-16
}



int main(int argc, char **argv)
{
    
    if (argc>1) {
      setup_fm();
      setupDMA(argc>2?atof(argv[2]):103.3);
      playWav(argv[1], argc>3?atof(argv[3]):22050, argc>4);
    } else
      fprintf(stderr, "Usage:   program wavfile.wav [freq] [sample rate] [stereo]\n\nWhere wavfile is 16 bit 22.5kHz Stereo.  Set wavfile to '-' to use stdin.\nfreq is in Mhz (default 103.3)\nsample rate of wav file in Hz\n\nPlay an empty file to transmit silence\n");
    
    return 0;

} // main


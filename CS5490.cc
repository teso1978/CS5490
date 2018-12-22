#include <node.h>
#include <node_buffer.h>
#include <v8.h>

using namespace v8;
using namespace node;

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <pthread.h>
#include "wiringPi.h"
#


static void errormsg(const char *s)
{
	ThrowException(Exception::Error(String::New(s)));
}

const char* ToCString(const v8::String::Utf8Value& value) {
  return *value ? *value : "<string conversion failed>";
}

long elapsedTime(struct timespec start_time) {
	struct timespec end_time;
	clock_gettime(CLOCK_REALTIME, &end_time);
	long diffInNanos = ((end_time.tv_sec - start_time.tv_sec) * 1E9) + (end_time.tv_nsec - start_time.tv_nsec);
	return diffInNanos;
}

struct timespec timer_start() {
	struct timespec start_time;
	clock_gettime(CLOCK_REALTIME, &start_time);
	return start_time;
}

static int fd=0;
static int SAMPLE_SIZE = 10;


/*
 * serialOpen:
 *	Open and initialise the serial port, setting all the right
 *	port parameters - or as many as are required - hopefully!
 *********************************************************************************
 */

int serialOpen (char *device, int baud)
{
	printf("opening device: %s at %d baud\n", device, baud);
  struct termios options ;
  speed_t myBaud ;
  int     status ;

  switch (baud)
  {
    case     50:	myBaud =     B50 ; break ;
    case     75:	myBaud =     B75 ; break ;
    case    110:	myBaud =    B110 ; break ;
    case    134:	myBaud =    B134 ; break ;
    case    150:	myBaud =    B150 ; break ;
    case    200:	myBaud =    B200 ; break ;
    case    300:	myBaud =    B300 ; break ;
    case    600:	myBaud =    B600 ; break ;
    case   1200:	myBaud =   B1200 ; break ;
    case   1800:	myBaud =   B1800 ; break ;
    case   2400:	myBaud =   B2400 ; break ;
    case   9600:	myBaud =   B9600 ; break ;
    case  19200:	myBaud =  B19200 ; break ;
    case  38400:	myBaud =  B38400 ; break ;
    case  57600:	myBaud =  B57600 ; break ;
    case 115200:	myBaud = B115200 ; break ;
    case 230400:	myBaud = B230400 ; break ;
    case 460800:	myBaud = B460800 ; break ;
	case 500000:	myBaud = B500000 ; break ;

    default:
      return -2 ;
  }

  if ((fd = open (device, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK)) == -1)
    return -1 ;

  fcntl (fd, F_SETFL, O_RDWR) ;

// Get and modify current options:

  tcgetattr (fd, &options) ;

    cfmakeraw   (&options) ;
    cfsetispeed (&options, myBaud) ;
    cfsetospeed (&options, myBaud) ;

    options.c_cflag |= (CLOCAL | CREAD) ;
    options.c_cflag &= ~PARENB ;
    options.c_cflag &= ~CSTOPB ;
    options.c_cflag &= ~CSIZE ;
    options.c_cflag |= CS8 ;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG) ;
    options.c_oflag &= ~OPOST ;

    options.c_cc [VMIN]  =   4000 ;
    options.c_cc [VTIME] = 2 ;	// two second (2 deciseconds)

  tcsetattr (fd, TCSANOW | TCSAFLUSH, &options) ;

  ioctl (fd, TIOCMGET, &status);

  status |= TIOCM_DTR ;
  status |= TIOCM_RTS ;

  ioctl (fd, TIOCMSET, &status);

  usleep (10000) ;	// 10mS

  printf("opened device: %s\n", device);
  return fd ;
}


/*
 * serialFlush:
 *	Flush the serial buffers (both tx & rx)
 *********************************************************************************
 */

Handle<Value> Flush (const Arguments& args)
{
	if (fd != 0)
		tcflush (fd, TCIOFLUSH);

	return v8::Integer::New(0);
}

static int errs=0;
int _Send(char * tx, int txLen, char * rx, int rxLen, bool verbose=false) 
{
	int rcount = 0;
	if (fd == 0) {
        errormsg("device not open");
    }
	//printf("before");
    // write to serial port
	if (txLen < 100) 
	{
		int wcount = write(fd, tx, txLen);
		if (wcount < 0) {
			errormsg("can't write to serial port");
			printf("write error");
		}
	}
	else
	{
		int pos =0, cnt=0;
		while (pos < txLen)
		{
			int l = txLen - pos;
			if (l > 500)
				l=500;
			write(fd, tx+pos, l);
			//printf("wrote: %d\n", l);
		
again2:
			if((rcount = read(fd, rx+pos, l )) < 0) 
			{

				if (errno == EINTR) {
					errs++;
					goto again2;
				}

				printf("read err: %s   ret: %d\n", strerror(errno), rcount);
				errormsg("can't read from serial port");
			} 
			cnt+=rcount;
			pos += l;

		}
		return cnt;
	}


	if (verbose) {
		printf("wrote: ");
		for (int i=0;i<txLen;i++) {
			printf("%02X", tx[i]);
			if (i%4==3)
				printf("-");
			else
				printf(" ");
		}
		printf("\n");
	}

	//int delayMs = txLen / 50;
	//printf("sleeping: %d ms", delayMs);
	//if (txLen > 100)
		//usleep(1000);
//	printf(" ... awake\n");
	
    // read output
    
    if (rxLen > 0)
    {
		if (verbose)
			printf("attempting to read %d bytes\n", rxLen);
again:
        if((rcount = read(fd, rx, rxLen )) < 0) 
		{
			if (errno == EINTR) {
				errs++;
				goto again;
			}
      
			printf("read err: %s   ret: %d\n", strerror(errno), rcount);
            errormsg("can't read from serial port");
		}
		
		if (verbose) {
			printf("read: ");
			for (int i=0;i<rcount;i++) {
				printf("%02X", rx[i]);
				if (i%3==2)
					printf("-");
				else
					printf(" ");
			}
			printf("\n");
		}
    }

    return rcount;
}


// Send Function
Handle<Value> Send(const Arguments& args) {
    
	HandleScope scope;
    
	if(args.Length() < 1 || args.Length() > 3)
		errormsg("Invalid number of parameters.  (inBuf, outBuf, verbose)");

	if (fd == 0) 
        errormsg("Device is closed");
    
	// first arg is data to send
    char* txBuffer = NULL;
    size_t txLen = 0;
    Local<Object> txBufferObj;
    txBufferObj = args[0]->ToObject();
    txBuffer = Buffer::Data(txBufferObj);
    txLen = Buffer::Length(txBufferObj);

    // second argument is optional receive buffer
    char* rxBuffer = NULL;
	size_t rxLen = 0;

	if(args.Length() >= 2)
	{
		Local<Object> rxBufferObj;
		rxBufferObj = args[1]->ToObject();
		rxBuffer = Buffer::Data(rxBufferObj);
		rxLen = Buffer::Length(rxBufferObj);
	}

	int verbose = 0;
	if(args.Length() >= 3)
	{
		Local<Integer> i = Local<Integer>::Cast(args[2]);
		verbose= (int)(i->Int32Value());
	}

	int rcount = _Send(txBuffer, txLen, rxBuffer, rxLen, verbose != 0);
	return v8::Integer::New(rcount);
}

static char * isrResultBuffer;
static int isrSampleCount;
static int isrMaxSampleCount;
static struct timespec isrStartTime;

static char txReadAndClearIsr[17];
static char txReadIsr[5];
static char rxIsr[9];
static int ISR_PIN = -1;

void DisableInterrupts()
{
	// disable interrupts
	char txDisable[] = { 0x80, 0x43, 0x00, 0x00, 0x00};
	_Send(txDisable, 5, NULL, 0);
}

void EnableInterrupts(char * buffer, int maxSamples)
{
	isrSampleCount = 0;
	isrMaxSampleCount = maxSamples;
	isrResultBuffer = buffer;

	// enable interrupts and select page 16 for inst reads
	char txEnable[] = { 0x80, 0x43, 0x00, 0x00, 0x40, 0x90};
	_Send(txEnable, 6, NULL, 0);
}

Handle<Value> Open(const Arguments& args) 
{
	 if(args.Length() != 2)
		errormsg("Invalid input - must be two arguments");

	 if(!args[0]->IsString())
		 errormsg("Invalid input - first argument must be device name");

	 if (!args[1]->IsInt32())
		 errormsg("Invalid input - second argument must be baud rate");

    int ret = 0;
    if (fd != 0) 
		close(fd);
	
	String::Utf8Value str(args[0]);
	char* device = (char*) ToCString(str);

	Local<Integer> i = Local<Integer>::Cast(args[1]);
    int baud = (int)(i->Int32Value());
		
	fd = serialOpen(device, baud);
	if (fd < 0) 
	{
		printf("can't open device: %s at %d bps\n", device, baud);
		errormsg("can't open device");
	}
    
	/*
	// try a command to see if everything is working
	int cmd[8];

	cmd[0]=0;
	cmd[1]=0;
	cmd[2]=16;
	cmd[3]=0;
	cmd[4]=0;
	cmd[5]=7;
	cmd[6]=16;
	cmd[7]=49;

	// build actual command buffer
	char * buf = (char*)malloc(8*2);
	char * res = (char*)malloc(8/2*3);
	for (int i=0;i<8/2; i++) {
		buf[i*4] = 0x80 + cmd[i*2];
		buf[i*4+1] = buf[i*4+2] = buf[i*4+3] = cmd[i*2+1];
	}


	memset(res, 0, 8/2*3);

    int read1 = WriteReadSerial(buf, 8*2, res, 8/2*3); 
	for (int i=0;i<8/2;i++)
		printf("read %d bytes: %2X-%2X-%2X\n", read1, res[i*3], res[(i*3)+1], res[(i*3)+2]);

	free(buf);
	free(res);
	*/
    return v8::Integer::New(ret);
}

Handle<Value>  Close(const Arguments& args) {
    int ret = 0;
    //Close Device
    if (fd != 0)
        ret = close(fd);
    fd=0;

    return v8::Integer::New(ret);
}

int ReadCommand(char * buffer, char page, char address, bool first = false)
{
	// Reads interfer with previous running reads so we need to pad the command with 
	// three page select commands before we start the next read to ensure the previous
	// read is complete
	int i=0;
	buffer[i++] = 0x80 + page;
	if (!first) {
		buffer[i++] = 0x80 + page;
		buffer[i++] = 0x80 + page;
	}
	buffer[i++] = address;

	return i;
}

int WriteCommand(char * buffer, char page, char address, char byte0, char byte1, char byte2)
{
	buffer[0] = 0x80 + page;		// page select
	buffer[1] = 0x80 + page;
	buffer[2] = 0x80 + page;
	buffer[3] = 0x40 + address;		// write to address
	buffer[4] = byte0;				// first byte
	buffer[5] = byte1;				// second byte
	buffer[6] = byte2;				// third byte

	return 7;
}

Handle<Value> Time(const Arguments& args) {
	struct timespec startTime;
	long elapsed=0, min=999999999, max = 0, last=0;

	startTime = timer_start();

	int iterations = 100000;
	for (int i=0;i<iterations; i++)
	{
		elapsed = elapsedTime(startTime);
	}
	printf("Total time for %d iterations of raw elapsedTime was %d ns (%d ns per iteration)\n", iterations, elapsed, elapsed / iterations);


	iterations = 100000;
	startTime = timer_start();
	for (int i=0;i<iterations; i++)
	{
		elapsed = (elapsedTime(startTime)) - last;
		if (elapsed > max)
			max = elapsed;
		if (elapsed < min)
			min = elapsed;
		last = elapsed;
	}
	elapsed = elapsedTime(startTime);

	printf("Total time for %d iterations of elapsedTime was %d ns (%d ns per iteration), min: %d, max: %d\n", iterations, elapsed, elapsed / iterations, min, max);

	// time read register operation
	char select0[1];
	select0[0] = 0x80;
	_Send(select0, 1, NULL, 0);  // select page 0

	char txRead1[1];   txRead1[0] = 23;
	char rxRead[3];
	errs=0;
	iterations = 1000;
	startTime = timer_start();
	for (int i=0;i<iterations; i++)
	{
		if (_Send(txRead1, 1, rxRead, 3) != 3)  // send 1 and recieve 3 bytes
			errormsg("wrong num returned");
	}
	elapsed = elapsedTime(startTime) / 1000;

	printf("Total time for %d iterations of reading 1 register was %d us (%d us per iteration) errs: %d\n", iterations, elapsed, elapsed / iterations, errs);


	char rx[12];  
	char txRead[17];
	ReadCommand(txRead, 0, 23);      // read status
	ReadCommand(txRead+4, 16, 2);    // read iInst
	ReadCommand(txRead+8, 16, 3);    // read vInst
	ReadCommand(txRead+12, 16, 61);  // read system time
	txRead[16] = 0x80;//  page select 0


	iterations = 1000;
	errs=0;
	int sendBitStart = 7, sendBits = 5, recBits = 6, recBitStart=3;
	min=999999999, max = 0, last=0;
	
	startTime = timer_start();
	for (int i=0;i<iterations; i++)
	{
		if (_Send(txRead+sendBitStart, sendBits, rx+recBitStart, recBits, false) != recBits)
			errormsg("wrong num returned");

		elapsed = (elapsedTime(startTime)/1000) - last;

		if (elapsed > max)
			max = elapsed;
		if (elapsed < min)
			min = elapsed;
		last = elapsed;
	}
	long telapsed = elapsedTime(startTime)/1000;

	printf("Total time for %d iterations of reading 2 registers together was %d us (%d us per iteration), min: %d, max: %d, errs: %d\n", iterations, telapsed, telapsed / iterations, min, max, errs);

	iterations = 1000;
	errs=0;
	sendBitStart = 7, sendBits = 5, recBits = 6, recBitStart=3;
	min=999999999, max = 0, last=0;
	startTime = timer_start();
	for (int i=0;i<iterations; i++)
	{
		if (_Send(txRead+7, 1, rx, 3) != 3)
			errormsg("wrong num returned");
		if (_Send(txRead+11, 1, rx, 3) != 3)
			errormsg("wrong num returned");
		elapsed = (elapsedTime(startTime)/1000) - last;

		if (elapsed > max)
			max = elapsed;
		if (elapsed < min)
			min = elapsed;
		last = elapsed;
	}
	elapsed = elapsedTime(startTime)/1000;

	printf("Total time for %d iterations of reading 2 registers separately was %d us (%d us per iteration), min: %d, max: %d, errs: %d\n", iterations, elapsed, elapsed / iterations, min, max, errs);

	iterations = 1000;
	errs=0;
	min=999999999, max = 0, last=0;
	startTime = timer_start();
	for (int i=0;i<iterations; i++)
	{
		if (_Send(txRead, 16, rx, 12) != 12)
			errormsg("wrong num returned");
		elapsed = (elapsedTime(startTime)/1000) - last;

		if (elapsed > max)
			max = elapsed;
		if (elapsed < min)
			min = elapsed;
		last = elapsed;
	}
	elapsed = elapsedTime(startTime)/1000;

	printf("Total time for %d iterations of reading 4 registers together was %d us (%d us per iteration), min: %d, max: %d, errs: %d\n", iterations, elapsed, elapsed / iterations, min, max, errs);

	iterations = 1000;
	errs=0;
	min=999999999, max = 0, last=0;
	startTime = timer_start();
	for (int i=0;i<iterations; i++)
	{
		if (_Send(txRead+3, 2, rx, 3) != 3)
			errormsg("wrong num returned");
		if (_Send(txRead+7, 1, rx, 3) != 3)
			errormsg("wrong num returned");
		if (_Send(txRead+11, 1, rx, 3) != 3)
			errormsg("wrong num returned");
		if (_Send(txRead+15, 2, rx, 3) != 3)
			errormsg("wrong num returned");
		elapsed = (elapsedTime(startTime)/1000) - last;

		if (elapsed > max)
			max = elapsed;
		if (elapsed < min)
			min = elapsed;
		last = elapsed;
	}
	elapsed = elapsedTime(startTime)/1000;

	printf("Total time for %d iterations of reading 4 registers separately was %d us (%d us per iteration), min: %d, max: %d, errs: %d\n", iterations, elapsed, elapsed / iterations, min, max, errs);


	char * txbuf = (char*)malloc(4000);
	char * rxbuf = (char*)malloc(3000);

	for (int i=0;i<500;i++)
	{
		txbuf[i*8] = 2;
		txbuf[i*8+1] = txbuf[i*8+2] = txbuf[i*8+3] = 0x90;
		txbuf[i*8+4] = 3;
		txbuf[i*8+5] = txbuf[i*8+6] = txbuf[i*8+7] = 0x90;
	}


	startTime = timer_start();
	int ret = _Send(txbuf, 4000, rxbuf, 3000, false);
	if( ret != 3000) {
		printf("wrong number returned: %d", ret);
			errormsg("wrong num returned");
	}
	elapsed = elapsedTime(startTime)/1000;

	printf("Total time for writing 4000 bytes and reading 3000 bytes was %d us errs: %d\n", elapsed, errs);

}


/*
void *wait(void *t)
{
   int i;
   long tid;

   tid = (long)t;

   sleep(1);

   pthread_exit(NULL);
}

void ReadThread()
{
   pthread_t thread;
   pthread_attr_t attr;
   void *status;

   // Initialize and set thread joinable
   pthread_attr_init(&attr);
   pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

   char * buffer = (char*) malloc(

   int rc = pthread_create(&thread, NULL, wait, NULL);
      if (rc){
         printf("unabled to create thread\n");
         exit(-1);
      }
   
}
*/
Handle<Value> ReadCycle(const Arguments& args) {
	HandleScope scope;
	int ret = 0;
	struct timespec startTime, totalTime;
	long elapsed=0;
	bool verbose = false;

	if (args.Length() < 2 || args.Length() > 3)
		return ThrowException(Exception::TypeError(String::New("Expected 2 or 3 arguments - output buffer, cmd (1=current, 2=voltage, 3=both), (optional) int verbose")));

	totalTime = timer_start();

	// first arg is output buffer
	char* out_buffer = NULL;
	size_t out_length = -1;
	Local<Object> out_buffer_obj;
	out_buffer_obj = args[0]->ToObject();
	out_buffer = Buffer::Data(out_buffer_obj);
	out_length = Buffer::Length(out_buffer_obj);
	int MAX_RESULTS = out_length / SAMPLE_SIZE;

	if (!args[1]->IsInt32())
		 errormsg("Invalid input - second argument must be cmd flag");

	Local<Integer> i = Local<Integer>::Cast(args[1]);
    int command = (int)(i->Int32Value());

	if (args.Length() == 3 && args[2]->IsInt32())
		 verbose = ((int)(Local<Integer>::Cast(args[2])->Int32Value())) != 0;

	// build txBuffer
	char txRead[16];
	ReadCommand(txRead, 0, 23);      // read status
	ReadCommand(txRead+4, 16, 2);    // read iInst
	ReadCommand(txRead+8, 16, 3);    // read vInst
	ReadCommand(txRead+12, 16, 61);  // read system time
	//WriteCommand(txRead+16, 0, 23, 0x00, 0x00, 0x40);  // clear CRDY

	// each read command returns 3 bytes
	char rx[12];  
	memset(rx, 0, 12); // clear to start

	// start continuous conversions
	char txStart[1];
	txStart[0] = 0xD5;  // start continuous conversions
	//	txStart[0] = 0xD4;  // start single conversion

	// halt computations
	char txHalt[1];
	txHalt[0] = 0xD8;  

	// clear status register
	char txClear[5];
	txClear[0] = 0x80;       // select page 0
	txClear[1] = 0x40 + 23;  // write to address 23
	txClear[4] = 0xE5;       // 11100101
	txClear[3] = 0x55;       // 01010101
	txClear[2] = 0x7D;       // 01111101


	// clear status
	_Send(txClear, 5, NULL, 0);
	
	// start conversions
	_Send(txStart, 1, NULL, 0);
	_Send(txClear, 5, NULL, 0);

	usleep(5000);  // wait 5 ms for things to settle
	
	char select16[1];
	select16[0] = 0x90;
	_Send(select16, 1, NULL, 0);  // select page 16 for inst reads


	int init_reads = 0;
	long initialelapsed = elapsed;

	    // read instantaneuos values for full cycle
    startTime = timer_start();
    int hits=0, misses=0, samples=0, pre=0, post=0, total=0;
    //char last1=0, last2=0, last3=0;
	//int sendBitStart = 7, sendBits = 5, recBits = 6, recBitStart=3;
	int sendBitStart = 0, sendBits = 12, recBits = 9, recBitStart=0;
	long x1=0;
	//rx[2] = 0x40;
    do 
    {
		total++;
		// read status, *Inst and system time values
		ret = _Send(txRead+sendBitStart, sendBits, rx+recBitStart, recBits, false);
		if (ret != recBits)
			errormsg("can't read status");

		// nano secs
		elapsed = elapsedTime(startTime);
		
		// Check CRDY bit
		if (rx[2] & 0x40)
		{
			hits++;

			// copy out result
			if (samples < MAX_RESULTS)
			{
				if (x1 == 0) {
					// once first sample is recieved we don't need to read 
					if (command == 1)  // current only
						sendBitStart = 7, sendBits = 1, recBits =3, recBitStart=3;
					else if (command == 2)
						sendBitStart = 11, sendBits = 1, recBits =3, recBitStart=6;
					else
						sendBitStart = 7, sendBits = 5, recBits =6, recBitStart=3;
					//sendBitStart = 7, sendBits = 5, recBits = 6, recBitStart=3;
					//sendBitStart = 11, sendBits = 1, recBits =3, recBitStart=6;
					//x1 = elapsed/250000;
					x1 = elapsed;
				}
				
				//long x = (elapsed/250000) - x1;
				long x = (elapsed - x1);// /1000;  //convert to microseconds
				//printf("x: %d\n", (int)x);
				//memcpy(out_buffer+(8*samples), rx+3, 6);
				memcpy(out_buffer+(SAMPLE_SIZE*samples), rx+3, 6);
				memcpy(out_buffer+(SAMPLE_SIZE*samples)+6, &x, 4);
				//out_buffer[6+(SAMPLE_SIZE*samples)] = (char)((x >> 24) & 0xFF);
				//out_buffer[7+(SAMPLE_SIZE*samples)] = (char)((x >> 16) & 0xFF);
				//out_buffer[8+(SAMPLE_SIZE*samples)] = (char)((x >> 8) & 0xFF);
				//out_buffer[9+(SAMPLE_SIZE*samples)] = (char)(x & 0xFF);

				samples++;
			}
			else
			{
				sendBitStart = 0, sendBits = 16, recBits = 12, recBitStart=0;
				post ++;
			}
			
			// clear CRDY bit in status register
			//_Send(txClearCRDY, 5, NULL, 0);
		}
		else if (x1 != 0)
		{
			misses++;
		}
		else
		{
			pre++;
		}
    } while (!(rx[2] & 0x80) && elapsed < 3E9);

	elapsed = elapsedTime(startTime);

	if (txStart[0] == 0xD5)
	{
		// halt conversions
		_Send(txHalt, 1, NULL, 0);
	}

	long totalelapsed = elapsedTime(totalTime);

	if (verbose)
	{
		if (txStart[0] == 0xD5)
			printf("Using continuous conversions\n");
		else
			printf("Using single conversion\n");
		if (command==1)
			printf("Measuring current\n");
		else if (command==2)
			printf("Measuring voltage\n");
		else
			printf("Measuring current and voltage\n");
		printf("SAMPLE_SIZE is %d bytes\n", SAMPLE_SIZE);
		printf("Input buffer is %d bytes\n", out_length);
		printf("Will collect a maximum of %d samples\n", MAX_RESULTS);
		printf("Initial read cycles %d\n", init_reads);
		printf("Initial read cycle duration: %g ms\n", initialelapsed / 1000000.0);
		printf("Final read cycle pre: %d, hits: %d,  misses: %d,  post: %d, total: %d\n", pre, hits, misses, post, total);
		printf("Final read cycle samples collected: %d\n", samples);
		printf("Final read cycle duration: %g ms\n", elapsed / 1000000.0);
		printf("Total duration: %g ms\n", totalelapsed / 1000000.0);
	}  

    if (elapsed >= 3E9)
		return v8::Integer::New(-2);

    return v8::Integer::New(samples);
}


Handle<Value> ReadCycleWithInterrupts(const Arguments& args) {
	HandleScope scope;
	int ret = 0;
	struct timespec startTime, totalTime;
	long elapsed=0;
	bool verbose = false;

	if (args.Length() < 1 || args.Length() > 2)
		return ThrowException(Exception::TypeError(String::New("Expected 1 or 2 arguments - output buffer, (optional) int verbose")));

	totalTime = timer_start();

	// first arg is output buffer
	char* out_buffer = NULL;
	size_t out_length = -1;
	Local<Object> out_buffer_obj;
	out_buffer_obj = args[0]->ToObject();
	out_buffer = Buffer::Data(out_buffer_obj);
	out_length = Buffer::Length(out_buffer_obj);
	int MAX_RESULTS = out_length / SAMPLE_SIZE;

	if (args.Length() == 2 && args[1]->IsInt32())
		 verbose = ((int)(Local<Integer>::Cast(args[1])->Int32Value())) != 0;

	// clear status register
	char txClear[5];
	txClear[0] = 0x80;       // select page 0
	txClear[1] = 0x40 + 23;  // write to address 23
	txClear[4] = 0xE5;       // 11100101
	txClear[3] = 0x55;       // 01010101
	txClear[2] = 0x7D;       // 01111101

	// clear status
	_Send(txClear, 5, NULL, 0);
	
	// start single conversion
	char x[] = { 0xD4 };
	_Send(x, 1, NULL, 0);

	char txReadStatus[] = { 0x80, 23 };  // set page 0, read reg 23
	char rx[3];  

	bool doInterrupts = true;
	int loops=0;
	startTime = timer_start();
    do 
    {
		loops++;

		// read status
		ret = _Send(txReadStatus, 2, rx, 3);
		if (ret != 3)
			errormsg("can't read status");

    	elapsed = elapsedTime(startTime);

		// Check CRDY bit
		if (doInterrupts && (rx[2] & 0x40))
		{
			doInterrupts = false;

			// start interrupt handler
			EnableInterrupts(out_buffer, MAX_RESULTS);

			// sleep for 500 ms
			sleep(500);

			DisableInterrupts();
		}
    } while (!(rx[2] & 0x80) && elapsed < 3E9);

	elapsed = elapsedTime(startTime);

	long totalelapsed = elapsedTime(totalTime);

	if (verbose)
	{
		printf("Input buffer: %d bytes, sample size: %d bytes, max samples: %d\n", out_length, SAMPLE_SIZE, MAX_RESULTS);
		printf("Total loops: %d\n", loops);
		printf("Total samples collected: %d\n", isrSampleCount);
		printf("Read loop duration: %g ms\n", elapsed / 1000000.0);
		printf("Total duration: %g ms\n", totalelapsed / 1000000.0);
	}  

    if (elapsed >= 3E9)
		return v8::Integer::New(-2);

    return v8::Integer::New(isrSampleCount);
}


void IsrHandler1(void)
{
	// handle interrupt here (connect DO pin on chip to ISR_PIN gpio input pin on pi)
	int ret = _Send(txReadAndClearIsr, 17, rxIsr, 9, false);
	if (ret != 9) {
		fprintf (stderr, "can't read from Isr: %s\n", strerror (errno)) ;
		//errormsg("can't read from Isr");
	}
	else
	{
		// cache results
	}
}

void IsrHandler2(void)
{
	// handle interrupt here (connect DO pin on chip to ISR_PIN gpio input pin on pi)
	int ret = _Send(txReadIsr, 5, rxIsr, 6);
	if (ret != 6) {
		fprintf (stderr, "can't read from Isr: %s\n", strerror (errno)) ;
		//errormsg("can't read from Isr");
	}
	else
	{
		// cache results
		int chip_sample=0;
		long elapsed = 0;

		if (isrSampleCount == 0)
		{
			isrStartTime = timer_start();
		}
		else
		{
			elapsed = elapsedTime(isrStartTime);
			chip_sample = (elapsed/250000.0) + 0.5;  // round to nearest int
		}
		
		if (isrSampleCount >= isrMaxSampleCount || elapsed > 5E8)  // stop after buffer is full or .5 sec has elapsed 
		{
			DisableInterrupts();	
		}
		else
		{
			memcpy(isrResultBuffer+(8*isrSampleCount), rxIsr, 6);
			isrResultBuffer[6+(8*isrSampleCount)] = (char)(chip_sample & 0xFF);
			isrResultBuffer[7+(8*isrSampleCount)] = (char)((chip_sample >> 8) & 0xFF);
			isrSampleCount++;
		}
	}
}


// GPIO stuff
Handle<Value> PinMode(const Arguments& args) {
    
	if (args.Length() != 2)
		return ThrowException(Exception::TypeError(String::New("Expected 2 arguments - pinNum and direction (in=0, out=1)")));

	int pin = (int)(Local<Integer>::Cast(args[0])->Int32Value());
	int mode = (int)(Local<Integer>::Cast(args[1])->Int32Value());

	pinMode(pin, mode);

    return v8::Integer::New(0);
}

Handle<Value> DigitalWrite(const Arguments& args) {
    if (args.Length() != 2)
		return ThrowException(Exception::TypeError(String::New("Expected 2 arguments - pinNum and value (0 or 1)")));

	int pin = (int)(Local<Integer>::Cast(args[0])->Int32Value());
	int value = (int)(Local<Integer>::Cast(args[1])->Int32Value());

	digitalWrite(pin, value);

    return v8::Integer::New(0);
}

Handle<Value> DigitalPulse(const Arguments& args) {
    if (args.Length() != 4)
		return ThrowException(Exception::TypeError(String::New("Expected 4 arguments - pinNum, value1, value2 and delay time in microseconds")));

	int pin = (int)(Local<Integer>::Cast(args[0])->Int32Value());
	int value1 = (int)(Local<Integer>::Cast(args[1])->Int32Value());
	int value2 = (int)(Local<Integer>::Cast(args[2])->Int32Value());
	int delay = (int)(Local<Integer>::Cast(args[3])->Int32Value());

	
	digitalWrite(pin, value1);
	printf("pin: %d set to: %d\n", pin, value1);

	delayMicroseconds(delay);
	digitalWrite(pin, value2);
	printf("pin: %d set to: %d\n", pin, value2);

    return v8::Integer::New(0);
}

Handle<Value> InitializeISR(const Arguments& args) {
    if (args.Length() != 1)
		return ThrowException(Exception::TypeError(String::New("Expected 1 argument, the interrupt pin number")));

	if (ISR_PIN == -1)
	{
		ISR_PIN = (int)(Local<Integer>::Cast(args[0])->Int32Value());

		if (wiringPiISR(ISR_PIN, INT_EDGE_FALLING, &IsrHandler2) < 0)
		{
			fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno)) ;
			errormsg("Unable to setup ISR");
		}

		int pos = 0;
		pos += ReadCommand(txReadAndClearIsr, 16, 2, true);    // read iInst
		pos += ReadCommand(txReadAndClearIsr+pos, 16, 3);    // read vInst
		pos += ReadCommand(txReadAndClearIsr+pos, 16, 61);  // read system time
		WriteCommand(txReadAndClearIsr+pos, 0, 23, 0x00, 0x00, 0x40);  // clear CRDY

		txReadIsr[0] = 0x02;   // read iInst
		txReadIsr[1] = txReadIsr[2] = txReadIsr[3] = 0x90;   // select page 16
		txReadIsr[4] = 0x03;   // read vInst
		
	}

    return v8::Integer::New(0);
}

/*
    Constructor
*/
void init(Handle<Object> exports) {

	exports->Set(String::NewSymbol("ReadCycleWithInterrupts"), FunctionTemplate::New(ReadCycleWithInterrupts)->GetFunction());
    exports->Set(String::NewSymbol("ReadCycle"), FunctionTemplate::New(ReadCycle)->GetFunction());
    exports->Set(String::NewSymbol("Open"), FunctionTemplate::New(Open)->GetFunction());
    exports->Set(String::NewSymbol("Close"), FunctionTemplate::New(Close)->GetFunction());
    exports->Set(String::NewSymbol("Send"), FunctionTemplate::New(Send)->GetFunction());
	exports->Set(String::NewSymbol("Flush"), FunctionTemplate::New(Flush)->GetFunction());
	exports->Set(String::NewSymbol("PinMode"), FunctionTemplate::New(PinMode)->GetFunction());
	exports->Set(String::NewSymbol("DigitalWrite"), FunctionTemplate::New(DigitalWrite)->GetFunction());
	exports->Set(String::NewSymbol("DigitalPulse"), FunctionTemplate::New(DigitalPulse)->GetFunction());
	exports->Set(String::NewSymbol("InitializeISR"), FunctionTemplate::New(InitializeISR)->GetFunction());
	exports->Set(String::NewSymbol("Time"), FunctionTemplate::New(Time)->GetFunction());

	if (wiringPiSetup() < 0)
	{
		fprintf (stderr, "Unable to setup wiringPi: %s\n", strerror (errno)) ;
		errormsg("Unable to setup wiringPi");
	}
}


NODE_MODULE(simpleuart, init)
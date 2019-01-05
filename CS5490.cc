#include <node_buffer.h>
#include <unistd.h>
#include <sys/time.h>
#include "wiringPi.h"
#include "wiringSerial.h"
#include "nan.h"

using namespace v8;
using namespace node;

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

static int fd = 0;
static int SAMPLE_SIZE = 10;

static char* Timestamp()
{
	static char ts[30];
	time_t ltime = time(NULL);
	struct tm * tm = localtime(&ltime);
	static struct timeval _t;
	static struct timezone tz;
	gettimeofday(&_t, &tz);

	sprintf(ts, "%04d-%02d-%02d %02d:%02d:%02d.%06d", tm->tm_year + 1900, tm->tm_mon, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec, (int)_t.tv_usec);
	return ts;
}

static void errormsg(const char *s)
{
	fprintf(stderr, "(%s) Error: %s\n", Timestamp(), s);
	Nan::ThrowError(s);
}

struct timespec timer_start() {
	struct timespec start_time;
	clock_gettime(CLOCK_REALTIME, &start_time);
	return start_time;
}

long elapsedTime(struct timespec start_time) {
	struct timespec end_time;
	clock_gettime(CLOCK_REALTIME, &end_time);
	long diffInNanos = ((end_time.tv_sec - start_time.tv_sec) * 1E9) + (end_time.tv_nsec - start_time.tv_nsec);
	return diffInNanos;
}

int _Send(char * tx, int txLen, char * rx, int rxLen, bool verbose=false) 
{
	int rcount = 0;
	if (fd == 0) {
        errormsg("device not open");
    }

    // write to serial port
	int wcount = write(fd, tx, txLen);
	if (wcount < 0) {
		errormsg("can't write to serial port");
		printf("write error");
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

    // read output
    if (rxLen > 0)
    {
		//if (verbose)
		//	printf("attempting to read %d bytes\n", rxLen);
        if((rcount = read(fd, rx, rxLen )) < 0) 
		{
			printf("read err: %s   ret: %d\n", strerror(errno), rcount);
            errormsg("can't read from serial port");
		}

		if (verbose) {
			printf("read %d bytes: ", rcount);
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

const char* ToCString(const v8::String::Utf8Value& value) {
	return *value ? *value : "<string conversion failed>";
}

void Close(const Nan::FunctionCallbackInfo<v8::Value>& info) {

	//Close Device
	if (fd != 0) {
		serialClose(fd);
		printf("Closed device\n");
	}
	fd = 0;

	info.GetReturnValue().Set(Nan::New<Number>(0));
}

void Open(const Nan::FunctionCallbackInfo<v8::Value>& info)
{
	int ret = 0;

	if(info.Length() != 2)
		errormsg("Invalid input - must be two arguments");

	 if(!info[0]->IsString())
		 errormsg("Invalid input - first argument must be device name");

	 if (!info[1]->IsInt32())
		 errormsg("Invalid input - second argument must be baud rate");

    Close(info);
	
	Isolate* isolate = info.GetIsolate();
	String::Utf8Value str(isolate, info[0]);
	char* device = (char*)ToCString(str);
	
	int baud = Nan::To<int32_t>(info[1]).FromJust();

	fd = serialOpen(device, baud);
	if (fd < 0) 
	{
		printf("can't open device: %s at %d bps\n", device, baud);
		errormsg("can't open device");
	}

	struct termios options;

	tcgetattr (fd, &options) ;   // Read current options
	options.c_cc [VMIN]  = 100 ;
    options.c_cc [VTIME] = 1 ;	// one second (10 deciseconds)
	tcsetattr (fd, TCSANOW, &options) ;   // Set new options
    
	printf("opened device: %s at %d bps\n", device, baud);

	info.GetReturnValue().Set(Nan::New<Number>(ret));
}

void _Instruction(char instruction)
{
	char tx[] = { instruction };
	_Send(tx, 1, NULL, 0);
}

void _WriteRegister(char page, char registerNum, int value)
{
	char pageSelect = 0x80 + page;
	char WriteCommand = 0x40 + registerNum;
	char data1 = value & 0xFF;
	char data2 = (value >> 8) & 0xFF;
	char data3 = (value >> 16) & 0xFF;
	char tx[] = {pageSelect, WriteCommand, data1, data2, data3 };

	_Send(tx, 5, NULL, 0);

	return;
}


int _ReadRegister(char page, char registerNum)
{
	char pageSelect = 0x80 + page;
	char readCommand = registerNum;
	char tx[] = {pageSelect, readCommand };
	char rx[3];
	
	_Send(tx, 2, rx, 3);

	return (*(rx + 2) << 16) + (*(rx + 1) << 8) + *rx;
}

int ReadSamples(char * resultBuffer, int maxSampleCount, int tSettle)
{
	// Select page 16
	char t[] = { 0x90 };
	_Send(t, 1, NULL, 0);

	char tx[] = { 
			0x02, 0xFF, 0xFF, 0xFF,	// read inst current
			0x03  					// read inst voltage
		};
	char rx[SAMPLE_SIZE];

	struct timespec startTime;
	int sampleCount = 0;
	while (sampleCount < maxSampleCount)  
	{
		char * buf = rx;
		if (sampleCount >= tSettle)
			buf = resultBuffer + ((sampleCount-tSettle)*SAMPLE_SIZE);

		int ret = _Send(tx, 5, buf, 6);

		if (ret >= 0) {

			long elapsed = 0;

			if (sampleCount == 0)
				startTime = timer_start();
			else
				elapsed = elapsedTime(startTime);

			memcpy(buf + 6, &elapsed, 4);   // timestamp in ns
			sampleCount++;
		}

		// wait at least 0.25 ms so we don't read the same sample twice
		//while (elapsedTime(start) < 250000)
		//{
			//spin
			//usleep(15);
		//}
	}
	return sampleCount-tSettle;
}

void ReadCycle(const Nan::FunctionCallbackInfo<v8::Value>& info)
{
	// make sure device is open    
	if (fd == 0) {
		errormsg("Must call Open first");
	}

	if (info.Length() != 1)
		errormsg("Expected 1 argument - buffer");

	char tx[] = { 
		0x18, 					  // halt conversions
		0x80, 					  // select page 0
		0x57, 0xFF, 0xFF, 0xFF,   // clear status 
		0xD4 					  // start single conversion
	};

	Isolate* isolate = info.GetIsolate();
	Local<Context> context = isolate->GetCurrentContext();
  	Local<Object> out_buffer_obj = info[0]->ToObject(context).ToLocalChecked();

	char* out_buffer = NULL;
	size_t out_length = -1;
	out_buffer = Buffer::Data(out_buffer_obj);
	out_length = Buffer::Length(out_buffer_obj);
	int maxSamples = out_length / SAMPLE_SIZE;

	// Get sample count and determine expected cycle time
	int sampleCount = _ReadRegister(16, 51);
	int tSettle = _ReadRegister(16, 57);
	
	// start conversion
	int ret = _Send(tx, 11, NULL, 0);
	if (ret < 0)
		errormsg("clear failed");

	// read instantaneous values 
	int samplesRead = ReadSamples(out_buffer, MIN(maxSamples+tSettle, sampleCount), tSettle);

	// Poll status until DRDY is set
	while (!(_ReadRegister(0, 23) & 0x800000))
	{
		// spin
		usleep(100);
	}
	return info.GetReturnValue().Set(Nan::New<Number>(samplesRead));
}

void PinMode(const Nan::FunctionCallbackInfo<v8::Value>& info) {

	if (info.Length() != 2)
		errormsg("Expected 2 arguments - pinNum and direction (in=0, out=1)");

	int pin = Nan::To<int>(info[0]).FromJust();
	int mode = Nan::To<int>(info[1]).FromJust();

	pinMode(pin, mode);

	info.GetReturnValue().Set(Nan::New<Number>(0));
}

void DigitalWrite(const Nan::FunctionCallbackInfo<v8::Value>& info)
{
	if (info.Length() != 2)
		errormsg("Expected 2 arguments - pinNum and value (0 or 1)");

	int pin = Nan::To<int>(info[0]).FromJust();
	int value = Nan::To<int>(info[1]).FromJust();

	digitalWrite(pin, value);

	info.GetReturnValue().Set(Nan::New<Number>(0));
}

void DigitalPulse(const Nan::FunctionCallbackInfo<v8::Value>& info)
{
	if (info.Length() != 4)
		errormsg("Expected 4 arguments - pinNum, value1, value2 and delay time in microseconds");

	int pin = Nan::To<int>(info[0]).FromJust();
	int value1 = Nan::To<int>(info[1]).FromJust();
	int value2 = Nan::To<int>(info[2]).FromJust();
	int delay = Nan::To<int>(info[3]).FromJust();

	digitalWrite(pin, value1);

	delayMicroseconds(delay);
	digitalWrite(pin, value2);

	info.GetReturnValue().Set(Nan::New<Number>(0));
}

void Flush (const Nan::FunctionCallbackInfo<v8::Value>& info)
{
	if (fd != 0)
		serialFlush(fd);

	info.GetReturnValue().Set(Nan::New<Number>(0));
}

void ReadRegister(const Nan::FunctionCallbackInfo<v8::Value>& info)
{
	if (info.Length() != 2)
		errormsg("Expected 2 arguments - page and register");
	
	if (!info[0]->IsInt32())
		 errormsg("Invalid input - first argument must be page number");

	if (!info[1]->IsInt32())
		 errormsg("Invalid input - second argument must be register number");

	// make sure device is open    
	if (fd == 0) 
		errormsg("Must call Open first");

	int page = Nan::To<int32_t>(info[0]).FromJust();
	int reg = Nan::To<int32_t>(info[1]).FromJust();

	int ret = _ReadRegister(page, reg);

	return info.GetReturnValue().Set(Nan::New<Number>(ret));
}

void WriteRegister(const Nan::FunctionCallbackInfo<v8::Value>& info)
{
	if (info.Length() != 3)
		errormsg("Expected 3 arguments - page, register, value");
	
	if (!info[0]->IsInt32())
		 errormsg("Invalid input - first argument must be page number");

	if (!info[1]->IsInt32())
		 errormsg("Invalid input - second argument must be register number");

	if (!info[2]->IsInt32())
		 errormsg("Invalid input - third argument must be value");

	// make sure device is open    
	if (fd == 0) 
		errormsg("Must call Open first");

	int page = Nan::To<int32_t>(info[0]).FromJust();
	int reg = Nan::To<int32_t>(info[1]).FromJust();
	int value = Nan::To<int32_t>(info[2]).FromJust();

	_WriteRegister(page, reg, value);

	return info.GetReturnValue().Set(Nan::New<Number>(0));
}

void Instruction(const Nan::FunctionCallbackInfo<v8::Value>& info)
{
	if (info.Length() != 1)
		errormsg("Expected 1 arguments - value");
	
	if (!info[0]->IsInt32())
		 errormsg("Invalid input - first argument must be instruction value");

	// make sure device is open    
	if (fd == 0) 
		errormsg("Must call Open first");

	char value = (char) Nan::To<int32_t>(info[0]).FromJust();
	_Instruction(value);

	return info.GetReturnValue().Set(Nan::New<Number>(0));
}

NAN_MODULE_INIT(InitAll) {

	Nan::Set(target, Nan::New("Open").ToLocalChecked(), Nan::GetFunction(Nan::New<v8::FunctionTemplate>(Open)).ToLocalChecked());
	Nan::Set(target, Nan::New("Close").ToLocalChecked(), Nan::GetFunction(Nan::New<v8::FunctionTemplate>(Close)).ToLocalChecked());
	Nan::Set(target, Nan::New("ReadRegister").ToLocalChecked(), Nan::GetFunction(Nan::New<v8::FunctionTemplate>(ReadRegister)).ToLocalChecked());
	Nan::Set(target, Nan::New("WriteRegister").ToLocalChecked(), Nan::GetFunction(Nan::New<v8::FunctionTemplate>(WriteRegister)).ToLocalChecked());
	Nan::Set(target, Nan::New("Instruction").ToLocalChecked(), Nan::GetFunction(Nan::New<v8::FunctionTemplate>(Instruction)).ToLocalChecked());
	
	Nan::Set(target, Nan::New("MeasureEnergy").ToLocalChecked(), Nan::GetFunction(Nan::New<v8::FunctionTemplate>(ReadCycle)).ToLocalChecked());
	Nan::Set(target, Nan::New("PinMode").ToLocalChecked(), Nan::GetFunction(Nan::New<v8::FunctionTemplate>(PinMode)).ToLocalChecked());
	Nan::Set(target, Nan::New("Flush").ToLocalChecked(), Nan::GetFunction(Nan::New<v8::FunctionTemplate>(Flush)).ToLocalChecked());
	Nan::Set(target, Nan::New("DigitalWrite").ToLocalChecked(), Nan::GetFunction(Nan::New<v8::FunctionTemplate>(DigitalWrite)).ToLocalChecked());
	Nan::Set(target, Nan::New("DigitalPulse").ToLocalChecked(), Nan::GetFunction(Nan::New<v8::FunctionTemplate>(DigitalPulse)).ToLocalChecked());

	if (wiringPiSetupPhys () < 0)
	{
		fprintf(stderr, "Unable to setup wiringPi: %s\n", strerror(errno));
		errormsg("Unable to setup wiringPi");
	}
}

NODE_MODULE(cs5490, InitAll)

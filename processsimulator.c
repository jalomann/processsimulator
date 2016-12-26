/*
** simulator_motor_server.c
** motor and process simulator who listens commands over network 
** connect with: telnet 192.168.1.2 9034
** or cityvalve scada
** www.jalomann.fi 2016
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <time.h>

#include <fcntl.h>
#include <termios.h>

#define INITIAL_SIZE 10
#define BUCKET_SIZE 5

#define PORT "9034"   // port we're listening on

// get sockaddr, IPv4 or IPv6:
void *get_in_addr(struct sockaddr *sa)
{
    if (sa->sa_family == AF_INET) {
        return &(((struct sockaddr_in*)sa)->sin_addr);
    }

    return &(((struct sockaddr_in6*)sa)->sin6_addr);
}

static int data_count_motor;		// how many ints we have stored 
static int data_size_motor;		// max amount
static int data_count_motor_input;	// how many ints we have stored 
static int data_size_motor_input;	// max amount
static int data_count_inverter;		// how many ints we have stored 
static int data_size_inverter;		// max amount
static int data_count_inverter_input;	// how many ints we have stored 
static int data_size_inverter_input;	// max amount
static int data_count_pump;		// how many ints we have stored 
static int data_size_pump;		// max amount
static int data_count_pump_input;	// how many ints we have stored 
static int data_size_pump_input;	// max amount
static int data_count_tank;		// how many ints we have stored 
static int data_size_tank;		// max amount
static int data_count_tank_input;	// how many ints we have stored 
static int data_size_tank_input;	// max amount

static int data_count_process;		// how many ints we have stored 
static int data_size_process;		// max amount

static struct motormodel *motortable;
static struct motorinputmodel *motorinputtable;

static struct invertermodel *invertertable;
static struct inverterinputmodel *inverterinputtable;

static struct pumpmodel *pumptable;
static struct pumpinputmodel *pumpinputtable;

static struct tankmodel *tanktable;
static struct tankinputmodel *tankinputtable;

static struct processmodel *processtable;

// xxx start struct motormodel
struct motormodel {
	int motorid;		// stat:  motor id
	int motorstatus;	// out:   0x1 ok, ready to op, temp, broken
	int motorstatusprev;	// out:   previous motor status
	float frequency;	// input: frequency
	float voltage;		// input: voltage
	float nomvoltage;	// stat:  nominal voltage
	float revspermin_act;	// calc:  actual speed
	float revspermin_nom;	// stat:  nominal revs per min (1000)
	float current;		// outp:  actual current
	float current_max;	// stat:  current max
	float cosfii;		// stat:  cos fii
	float torque;		// input: motor load Nm
	float power;		// calc:  motor load kW
	float runningtime;	// calc:  motor running time s
	float runningtimelimit;	// stat:  motor running time limit for calc s
	float standstilltime;	// calc:  motor standstill (cooling) s
	float temperature_act;	// calc:  motor temperature
	float temperature_al;	// stat:  motor temperature max (alarm)
	float temperature_bd;	// stat:  motor temperature max (break down)
	float temperaturerampup;	// stat: temperature delta C/s
	float rampup;		// stat:  speed up time s
	float rampdown;		// stat:  speed down time s
};
// motorstatus
// 0x00 stopped
// 0x01 running ok
// 0x02 
// 0x03 over current
// 0x11 temperature alarm
// 0x12 destroyed

// xxx start struct motormodel
struct motorinputmodel {
	int motorid;		// stat:  motor id
	int invid;		// stat:  feeding inverter id
	float frequency;	// input: frequency
	float voltage;		// input: voltage
	float torque;		// input: motor load Nm
	int processinput1;
	int processinput2;
	int processinput3;
};
// motorinput
// frequency
// voltage
// torque

// xxx start struct invertermodel
struct invertermodel {
	int inverterid;		// stat: motor id
	int inverterstatus;	// out:  see below
	int inverterstatusprev;	// out:  see below
	float setpoint;		// input:  0-100%
	float frequency;	// out:  frequency
	float voltage;		// out:  voltage V
	float voltagein;	// out:  voltage V
	float nomvoltage;	// stat: voltage V
	float current;		// out:  actual current
	float current_max;	// stat: current max
	float power;		// calc: output load kW
	float rampup;		// stat: speed up time s
	float rampdown;		// stat: speed down time s
};
// motorstatus
// 0x00 stopped
// 0x01 running ok
// 0x02 current limit
// 0x03 over current
// 0x11 tripped
// 0x12 destroyed

struct inverterinputmodel {
	int inverterid;		// stat:  inverter id
	float frequency;	// input: frequency
	float voltage;		// input: voltage
	float setpoint;		// input: setpoint -100..100
	int processinput1;
	int processinput2;
	int processinput3;
};
// motorinput
// frequency
// voltage
// torque

struct pumpmodel {
	int pumpid;		// stat: motor id
	int pumpstatus;		// out:  see below
	int pumpstatusprev;	// out:  see below
	float revspermin_act;	// input:  0-1000
	float revspermin_nom;	// stat: 0-1000
	float torque;		// out:  
	float flow;		// out:  flow 0-1000liters/s
	float flowmax;		// stat: flow   1000liters/s
	float runningtime;	// calc: 
	float temperature_act;	// calc: 
	float temperature_al;	// calc: 
	float temperature_bd;	// calc: 
};
// pumpstatus
// 0x00 stopped
// 0x01 running ok
// 0x03 over load
// 0x11 vibration
// 0x12 destroyed

struct pumpinputmodel {
	int pumpid;		// stat:  pump id
	float revspermin_act;	// input:  0-1000
	int processinput1;
	int processinput2;
	int processinput3;
};
// motorinput
// frequency
// voltage
// torque

struct tankmodel {
	int tankid;		// stat: motor id
	int tankstatus;		// out:  see below
	int tankstatusprev;	// out:  see below
	float flowin;		// out:  flow 0-1000liters/s
	float flowinmax;	// stat: flow   1000liters/s
	float flowout;		// out:  flow 0-1000liters/s
	float flowoutmax;	// stat: flow   1000liters/s
	float volume;		// calc: liters
	float volumemax;		// calc: liters
	float temperature_act;	// calc: 
	float temperature_al;	// calc: 
	float emptylimit;	// calc: 
	float normallimit;	// calc: 
	float fullwarninglimit;	// calc: 
	float leakinglimit;	// calc: 
};
// pumpstatus
// 0x00 empty
// 0x01 normal
// 0x03 full
// 0x11 warning
// 0x12 leaking

struct tankinputmodel {
	int tankid;		// stat:  tank id
	int pumpinid;		// stat:  pump id pumping in
	int pumpoutid;		// stat:  pump id pumping out
	int processinput1;
	int processinput2;
	int processinput3;
};

// xxx processmodel
struct processmodel {
	int processid;		// stat:  process id
	float value;		// calc:  value
	char text[24];		// stat:  process description
};


// xxx start vahennakierroksia. not used at the moment
int vahennakierroksia( struct motormodel *m) 
{
	m->revspermin_act--;
	// printf("sub. vahemman kierroksia.\n");
	return;
}

int addinvrevs (int id){
	struct inverterinputmodel input;
	struct invertermodel inverter;
	// int invid=0;
	// printf("More revs ... Juhuu %d \n", id);
	
	input = inverterinputtable[id];
	input.setpoint=input.setpoint + 0.9;
	if (input.setpoint>105) input.setpoint=105;
	inverterinputtable[id]=input;
	
	return 0;
	
}

int decinvrevs (int id){
	struct inverterinputmodel input;
	struct invertermodel inverter;
	// int invid=0;
	// printf("Less revs ... Even backwards %d \n", id);
	
	input = inverterinputtable[id];
	input.setpoint=input.setpoint - 0.8;
	if (input.setpoint<-105) input.setpoint=-105;
	inverterinputtable[id]=input;
	
	return 0;
	
}

int setinvsetpoint (int id, int sp){
	struct inverterinputmodel input;
	struct invertermodel inverter;
	
	input = inverterinputtable[id];
	input.setpoint=sp;
	if (input.setpoint<-105) input.setpoint=-105;
	inverterinputtable[id]=input;
	
	return 0;
	
}

int printoutinverter(int id) {
	struct invertermodel inverter;
	inverter = invertertable[id];
	printf("Inverter %3d SP     %5.1f     Hertz %5.1f Hz Voltage   %6.1f V  Current%5.1f A  Status  %2d ",
                id,
                inverter.setpoint,
                inverter.frequency, 
                inverter.voltage,
                inverter.current,
                inverter.inverterstatus);
	return 0;
}

int printoutmotor (int id){
	struct motormodel motor;
	// int invid=0;
	// printf("print out motor id ... %d\n", id);
	
	motor = motortable[id];
	
	printf("Motor    %3d Speed %6.1f rpm Current %4.1f A Temperature %4.1f °C RTime  %5.1f s  Status  %2d\n",
                id,
                motor.revspermin_act,
                motor.current,
                motor.temperature_act,
                motor.runningtime,
                motor.motorstatus);
        printf("Motor    %3d Power  %5.1f kW  Voltage %5.1f V Torque   %5.1f N   Message ",
                id,
                motor.power,
                motor.voltage,
                motor.torque);
	return 0;
	
}

int printoutpump(int id) {
	struct pumpmodel pump;
	pump = pumptable[id];
	printf("Pump     %3d Speed %6.1f rpm Flow:   %5.1f l/sec",
		id,
		pump.revspermin_act, 
		pump.flow);
	return 0;
}

int printouttank(int id) {
	struct tankmodel tank;
	tank = tanktable[id];
	printf("Tank     %3d Vol %8.1f l",
		id,
		tank.volume);
	return 0;
}

int printoutprocess(int id) {
	struct processmodel process;
	process = processtable[id];
	printf("%4d %4d %8.1f %-24s ",
		id,
		process.processid, 
		process.value, 
		process.text);
	return 0;
}

// prints out info row every 10 row
printinforow() {
	
	printf("set   freq   volt  curr sta |   rpm  curr   temp   po    volt     tor  run  sta   sta \n");
	printf("poin  uenc    age   inv tus |   mot  moto   erat   wer    age     que  time tus   hex \n");
	return 0;
}

// xxx start testi
int torquedynamization( float *f, int up) {
	
	if ( up > 0) {
		*f = (*f + ((*f) * 0.1));
	}
	else {
		*f = (*f - ((*f) * 0.1));
	}
	
        // print for debugging
	// printf("%4.1f ",f);
	
	return 0;
}

// xxx start pointertonemember as test
int pointertonemember(int *mstatus)
{
  printf(" %d",*mstatus);
	return 0;
}

// xxx start getch
char getch()
{
	//printf(" 1.");	
	fd_set input_fdset;
	int old_flags, old_flags2;
	struct termios term_attr;
	char c = 0;
	
	if (tcgetattr(STDIN_FILENO, &term_attr) != 0)
	{
		perror("getch:tcgetattr() failed!");
	}
	// printf("2.");
	old_flags = term_attr.c_lflag;
	term_attr.c_lflag &= ~(ICANON | ECHOE);
	
	if (tcsetattr(STDIN_FILENO, TCSAFLUSH, &term_attr) != 0)
	{
		perror("getch:tcsetattr() failed!");
	}
	// printf("3.");
	
	if (tcgetattr(STDOUT_FILENO, &term_attr) != 0)
	{
		perror("getch:tcgetattr() failed!");
	}
	old_flags2 = term_attr.c_lflag;
	term_attr.c_lflag = 0 ;
	// printf("4.");
	
	if (tcsetattr(STDOUT_FILENO, TCSAFLUSH, &term_attr) != 0)
	{
		perror("getch:tcsetattr() failed!");
	}
	
	// xxx
	while (1)
	{
		// printf("5.");
		struct timeval tv;
		tv.tv_sec = 1;
		tv.tv_usec = 500000;
		
		FD_ZERO(&input_fdset);
		FD_SET(STDIN_FILENO, &input_fdset);
                if (select(STDIN_FILENO + 1, &input_fdset, NULL, NULL, &tv) == -1)
		{
			perror("getch:select() failed!");
		}
		// printf("6.");
		// printf("getch: while: 6 comes after select (keypressed or timeout)\n");
		
		if (FD_ISSET(STDIN_FILENO, &input_fdset))
		{
			// printf("7.\n");
			// printf("getch: while: 7 comes after keypressed was pressed\n");
			
			// here is the actual reading and writes to c
			if (read(STDIN_FILENO, &c, 1) == -1)
			{
				perror("getch:read() failed!");
			}
			else
			{
				// printf("keypressed...\n");
				// printf("getch: while: 8 just before break. keypressed...\n");
				break;
			}
		}
		// printf("NOT keypressed...\n");
		break;
	}
	term_attr.c_lflag = old_flags;
	// printf("getch: 9 \n");
	
	if (tcsetattr(STDIN_FILENO, TCSAFLUSH, &term_attr) != 0)
	{
		perror("getch:tcsetattr() failed!");
	}
	// printf("getch: 10 \n");
	
	term_attr.c_lflag = old_flags2;
	if (tcsetattr(STDOUT_FILENO, TCSAFLUSH, &term_attr) != 0)
	{
		perror("getch:tcsetattr() failed!");
	}
	// printf("getch: 11 \n");
	
	return(c);
}


void updatemotortable(int motor, struct motormodel newdata){
	if (data_count_motor == data_size_motor ){
		data_size_motor += 10;
		// printf("%d\n",data_size_motor);
		if ((motortable = realloc(motortable, data_size_motor * sizeof(struct motormodel))) == NULL){
			printf("003 Uuups. Out of memory error when reallocating motor table.\n");
		}
	}
	motortable[motor] = newdata;
	data_count_motor++;
}

void updatemotorinputtable(int motor, struct motorinputmodel newdata){
	if (data_count_motor_input == data_size_motor_input ){
		data_size_motor_input += 10;
		// printf("%d\n",data_size_motor_input);
		if ((motorinputtable = realloc(motorinputtable, data_size_motor_input * sizeof(struct motorinputmodel))) == NULL){
			printf("003 Uuups. Out of memory error when reallocating motor input table.\n");
		}
	}
	motorinputtable[motor] = newdata;
	data_count_motor_input++;
}

void updateinvertertable(int inverter, struct invertermodel newdata){
	if (data_count_inverter == data_size_inverter ){
		data_size_inverter += 10;
		// printf("%d\n",data_size_inverter);
		if ((invertertable = realloc(invertertable, data_size_inverter * sizeof(struct invertermodel))) == NULL){
			printf("003 Uuups. Out of memory error when reallocating inverter table.\n");
		}
	}
	invertertable[inverter] = newdata;
	data_count_inverter++;
}

void updateinverterinputtable(int inverter, struct inverterinputmodel newdata){
	if (data_count_inverter_input == data_size_inverter_input ){
		data_size_inverter_input += 10;
		// printf("%d\n",data_size_inverter_input);
		if ((inverterinputtable = realloc(inverterinputtable, data_size_inverter_input * sizeof(struct inverterinputmodel))) == NULL){
			printf("003 Uuups. Out of memory error when reallocating inverter input table.\n");
		}
	}
	inverterinputtable[inverter] = newdata;
	data_count_inverter_input++;
}

void updatepumptable(int pump, struct pumpmodel newdata){
	if (data_count_pump == data_size_pump ){
		data_size_pump += 10;
		// printf("%d\n",data_size_pump);
		if ((pumptable = realloc(pumptable, data_size_pump * sizeof(struct pumpmodel))) == NULL){
			printf("003 Uuups. Out of memory error when reallocating pump table.\n");
		}
	}
	pumptable[pump] = newdata;
	data_count_pump++;
}

void updatepumpinputtable(int pump, struct pumpinputmodel newdata){
	if (data_count_pump_input == data_size_pump_input ){
		data_size_pump_input += 10;
		// printf("%d\n",data_size_pump_input);
		if ((pumpinputtable = realloc(pumpinputtable, data_size_pump_input * sizeof(struct pumpinputmodel))) == NULL){
			printf("003 Uuups. Out of memory error when reallocating pump input table.\n");
		}
	}
	pumpinputtable[pump] = newdata;
	data_count_pump_input++;
}

void updatetanktable(int tank, struct tankmodel newdata){
	if (data_count_tank == data_size_tank ){
		data_size_tank += 10;
		// printf("%d\n",data_size_tank);
		if ((tanktable = realloc(tanktable, data_size_tank * sizeof(struct tankmodel))) == NULL){
			printf("003 Uuups. Out of memory error when reallocating tank table.\n");
		}
	}
	tanktable[tank] = newdata;
	data_count_tank++;
}

void updatetankinputtable(int tank, struct tankinputmodel newdata){
	if (data_count_tank_input == data_size_tank_input ){
		data_size_tank_input += 10;
		// printf("%d\n",data_size_tank_input);
		if ((tankinputtable = realloc(tankinputtable, data_size_tank_input * sizeof(struct tankinputmodel))) == NULL){
			printf("003 Uuups. Out of memory error when reallocating tank table.\n");
		}
	}
	tankinputtable[tank] = newdata;
	data_count_tank_input++;
}


void updateprocesstable(int prid, struct processmodel newdata){
	if (data_count_process == data_size_process ){
		data_size_process += 10;
		// printf("%d\n",data_size_process);
		if (( processtable = realloc(processtable, data_size_process * sizeof(struct motormodel))) == NULL){
			printf("003 Uuups. Out of memory error when reallocating.\n");
		}
	}
	processtable[prid] = newdata;
	data_count_process++;
}

void calculateinverter(int id){
	struct invertermodel inverter;
	struct inverterinputmodel input;
	struct processmodel process;
	int process1=0;
	int process2=0;
	int process3=0;
	float processvalue1=0;
	float processvalue2=0;
	float processvalue3=0;

	if(0){
		printf("\n");
		printf("Inverter input\n");
		printf("id ... %d\n", id);
	}

	inverter = invertertable[id];
	input = inverterinputtable[id];
	
	inverter.voltagein=input.voltage;
	
	process1 = input.processinput1;
	process = processtable[process1];
	processvalue1=process.value;

	process2 = input.processinput2;
	process = processtable[process2];
	processvalue2=process.value;

	process3 = input.processinput3;
	process = processtable[process3];
	processvalue1=process.value;

	char charTime[10];
	unsigned int decAlarmTime;

	time_t currtime;
	char *malarm[20];
	malarm[0]="Stopped";
	malarm[1]="Running. Ok";
	malarm[3]="Current limit";
	malarm[11]="Over current";
	malarm[12]="Destroyed";

	if(0){
		printf("Process\n");
		printf("%d\n", process.processid);
		printf("%5.1f\n", process.value);
	}

	inverter.setpoint=input.setpoint;
	
	if(0){
		printf("inverter\n");
		printf("inverter.inverterid       ");
		printf("%5d\n", inverter.inverterid);		// stat: motor id
		printf("inverter.inverterstatus   ");
		printf("%5d\n", inverter.inverterstatus);	// out:  see below
		printf("inverter.setpoint         ");
		printf("%5.1f\n", inverter.setpoint);		// input:  0-100%
		printf("inverter.frequency        ");
		printf("%5.1f\n", inverter.frequency);		// out:  frequency
		printf("inverter.voltage          ");
		printf("%5.1f\n", inverter.voltage);		// out:  voltage V
		printf("inverter.nomvoltage       ");
		printf("%5.1f\n", inverter.nomvoltage);		// stat: voltage V
		printf("inverter.current          ");
		printf("%5.1f\n", inverter.current);		// out:  actual current
		printf("inverter.current_max      ");
		printf("%5.1f\n", inverter.current_max);	// stat: current max
		printf("inverter.power            ");
		printf("%5.1f\n", inverter.power);		// calc: motor load kW
		printf("inverter.rampup           ");
		printf("%5.1f\n", inverter.rampup);		// stat: speed up time s
		printf("inverter.rampdown         ");
		printf("%5.1f\n", inverter.rampdown);		// stat: speed down time s
	}

	inverter.frequency = (inverter.setpoint /2);
	inverter.voltage = (inverter.setpoint/100)*inverter.nomvoltage;
	inverter.current = (inverter.setpoint/100)*inverter.current_max;
	
	if ( inverter.current < 0) inverter.current = 0;

	if ( inverter.current < (inverter.current_max - 0.5)) {
		inverter.inverterstatus = 1; // back to normal
	}

	if ( inverter.current > (inverter.current_max - 0.1)) {
		inverter.inverterstatus = 11; // alarm
	}

	printoutinverter(id);

	if (inverter.inverterstatus != inverter.inverterstatusprev){
		// print out alarm time
		time(&currtime);  // tarvitaan. hakee varmaan ajan tohon currtimeen
		strftime(charTime,sizeof(charTime)-1,"%H%M%S",localtime(&currtime));
		decAlarmTime = atoi(charTime);			// alarm on time
		printf(" %6d", decAlarmTime);
		// print out alarm text
		printf (" %s",malarm[inverter.inverterstatus]);
	}

	inverter.inverterstatusprev = inverter.inverterstatus;
	invertertable[id] = inverter;
	
}

void calculatemotor(int id){
	struct invertermodel inverter;
	struct motorinputmodel input;
	struct motormodel motor;
	struct processmodel process;
	int invid=0;
	int process1=0;
	int process2=0;
	int process3=0;
	float processvalue1=0;
	float processvalue2=0;
	float processvalue3=0;

	time_t currtime;
	char charTime[10];
	unsigned int decAlarmTime;

	char *malarm[20];
	malarm[0]="Stopped";
	malarm[1]="Running. Ok";
	malarm[3]="Over current";
	malarm[11]="Temperature too high";
	malarm[12]="Destroyed";

	if (0){
		printf("motor id ... %d\n", id);
		printf("motor input\n");
	}


	motor = motortable[id];
	input = motorinputtable[id];
	
	process1 = input.processinput1;
	process = processtable[process1];
	processvalue1=process.value;
	
	invid = input.invid;
	inverter = invertertable[invid];
	motor.voltage = inverter.voltage;
	motor.frequency = inverter.frequency;
	motor.current = inverter.current;

	if (motor.current > motor.current_max) motor.current=motor.current_max;
	
	// motor.motorstatus = 1;
	motor.runningtime = motor.runningtime + 1.1;
	if (motor.runningtime > motor.runningtimelimit) motor.runningtime=motor.runningtimelimit;

	motor.revspermin_act = ((motor.frequency/50) * 1000);
	motor.power = motor.current * motor.voltage * motor.cosfii;
	// motor.power = (((motor.revspermin_act/9550)*motor.torque)*1000);
	// P[kW] = (rpm/9550)/T
	// *1000 -> W
	// motor.current = (motor.power / (motor.voltage * motor.cosfii));
	motor.temperature_act = ((motor.current * motor.runningtime)*0.1);
	if ( motor.temperature_act > motor.temperature_al ) {
		motor.motorstatus = 11; // alarm
	}
	if ( motor.temperature_act > motor.temperature_bd ) {
		motor.motorstatus = 12; // destroyed
	}

	if (0){
		printf("Motor\n");
	}


	// motor.temperature_act = motor.runningtime + 3;

	if(0){
		printf("inverter\n");
		printf("inverter.inverterid       ");
		printf("%5d\n", inverter.inverterid);		// stat: motor id
		printf("inverter.inverterstatus   ");
		printf("%5d\n", inverter.inverterstatus);	// out:  see below
		printf("inverter.setpoint         ");
		printf("%5.1f\n", inverter.setpoint);		// input:  0-100%
		printf("inverter.frequency        ");
		printf("%5.1f\n", inverter.frequency);		// out:  frequency
		printf("inverter.voltage          ");
		printf("%5.1f\n", inverter.voltage);		// out:  voltage V
		printf("inverter.nomvoltage       ");
		printf("%5.1f\n", inverter.nomvoltage);		// stat: voltage V
		printf("inverter.current          ");
		printf("%5.1f\n", inverter.current);		// out:  actual current
		printf("inverter.current_max      ");
		printf("%5.1f\n", inverter.current_max);	// stat: current max
		printf("inverter.power            ");
		printf("%5.1f\n", inverter.power);		// calc: motor load kW
		printf("inverter.rampup           ");
		printf("%5.1f\n", inverter.rampup);		// stat: speed up time s
		printf("inverter.rampdown         ");
		printf("%5.1f\n", inverter.rampdown);		// stat: speed down time s
	}

	if (0){
		printf("motor\n");
		printf("motor.motorid             ");
		printf("%5d\n", motor.motorid);			// stat: motor id
		printf("motor.motorstatus         ");
		printf("%5d\n", motor.motorstatus);		// out:  see below
		printf("motor.frequency           ");
		printf("%5.1f\n", motor.frequency);		// out:  frequency
		printf("motor.voltage             ");
		printf("%5.1f\n", motor.voltage);		// out:  voltage V
		printf("motor.nomvoltage          ");
		printf("%5.1f\n", motor.nomvoltage);		// stat: voltage V
		printf("motor.current             ");
		printf("%5.1f\n", motor.current);		// out:  actual current
		printf("motor.current_max         ");
		printf("%5.1f\n", motor.current_max);		// stat: current max
		printf("motor.power               ");
		printf("%5.1f\n", motor.power);			// calc: motor load kW
		printf("motor.runningtime         ");
		printf("%5.1f\n", motor.runningtime);		// stat: speed up time s
		printf("motor.temperature_act     ");
		printf("%5.1f\n", motor.temperature_act);	// stat: motor temperature
		printf("motor.temperature_al      ");
		printf("%5.1f\n", motor.temperature_al);	// stat: warning temperature
		printf("motor.temperature_bd      ");
		printf("%5.1f\n", motor.temperature_bd);	// stat: warning temperature
	}

	printoutmotor(id);	// print out the info

	if (motor.motorstatus != motor.motorstatusprev){
		// print out alarm time
		time(&currtime);  // tarvitaan. hakee varmaan ajan tohon currtimeen
		strftime(charTime,sizeof(charTime)-1,"%H%M%S",localtime(&currtime));
		decAlarmTime = atoi(charTime);			// alarm on time
		printf(" %6d", decAlarmTime);
		// print out alarm text
		printf (" %s",malarm[motor.motorstatus]);
	}
	if(0){
		
		printf("\n");
		printoutprocess(0);
		printf("\n");
		printoutprocess(1);
		printf("\n");
		printoutprocess(2);
		printf("\n");
		printoutprocess(3);
		printf("\n");
		printoutprocess(4);
		printf("\n");
		printoutprocess(5);
		printf("\n");
	}

	motor.motorstatusprev = motor.motorstatus;
	motortable[id] = motor;
	
	// printf("%5.1f\n", motor.voltage);		// out:  voltage V
	
}

void calculatepump(int id){
	struct pumpmodel pump;
	struct pumpinputmodel input;
	struct motormodel motor;
	struct processmodel process;
	int pumpid=0;
	int process1=0;
	int process2=0;
	int process3=0;
	float processvalue1=0;
	float processvalue2=0;
	float processvalue3=0;

	time_t currtime;
	char charTime[10];
	unsigned int decAlarmTime;

	char *malarm[20];
	malarm[0]="Stopped";
	malarm[1]="Running. Ok";
	malarm[3]="Over load";
	malarm[11]="Vibration";
	malarm[12]="Destroyed";

	if (0){
		printf("pump id ... %d\n", id);
		printf("pump input\n");
	}

	pump = pumptable[id];
	input = pumpinputtable[id];
	
	process1 = input.processinput1;
	process = processtable[process1];
	processvalue1=process.value;
	
	pumpid = input.pumpid;
	pump.revspermin_act = motor.revspermin_act;

	if (pump.revspermin_act>pump.revspermin_nom) pump.pumpstatus=11;
	
	pump.runningtime = pump.runningtime + 0.7;
	pump.flow=((pump.revspermin_act / pump.revspermin_nom) * pump.flowmax);
	pump.torque = pump.revspermin_act * pump.revspermin_act;

	if (0){
		printf("Pump\n");
	}

	// pump.temperature_act = pump.runningtime + 3;

	if (0){
		printf("pump\n");
		printf("pump.pumpid             ");
		printf("%5d\n", pump.pumpid);			// stat: pump id
		printf("pump.pumpstatus         ");
		printf("%5d\n", pump.pumpstatus);		// out:  see below
		printf("pump.frequency           ");
		printf("%5.1f\n", pump.revspermin_act);		// out:  frequency
		printf("pump.rpm max             ");
		printf("%5.1f\n", pump.revspermin_nom);		// out:  voltage V
		printf("pump.torque              ");
		printf("%5.1f\n", pump.torque);		// stat: voltage V
		printf("pump.flow                ");
		printf("%5.1f\n", pump.flow);		// out:  actual current
		printf("pump.runningtime         ");
		printf("pump.temperature_act     ");
		printf("%5.1f\n", pump.temperature_act);	// stat: pump temperature
		printf("pump.temperature_al      ");
		printf("%5.1f\n", pump.temperature_al);	// stat: warning temperature
		printf("pump.temperature_bd      ");
		printf("%5.1f\n", pump.temperature_bd);	// stat: warning temperature
	}

	printoutpump(id);	// print out the info

	if (pump.pumpstatus != pump.pumpstatusprev){
		// print out alarm time
		time(&currtime);  // tarvitaan. hakee varmaan ajan tohon currtimeen
		strftime(charTime,sizeof(charTime)-1,"%H%M%S",localtime(&currtime));
		decAlarmTime = atoi(charTime);			// alarm on time
		printf(" %6d", decAlarmTime);
		// print out alarm text
		printf (" %s",malarm[pump.pumpstatus]);
	}
	if(0){
		printf("\n");
		printoutprocess(0);
		printf("\n");
		printoutprocess(1);
		printf("\n");
		printoutprocess(2);
		printf("\n");
		printoutprocess(3);
		printf("\n");
		printoutprocess(4);
		printf("\n");
		printoutprocess(5);
		printf("\n");
	}

	pump.pumpstatusprev = pump.pumpstatus;
	pumptable[id] = pump;

}

void calculatetank(int id){
	struct tankmodel tank;
	struct pumpmodel pump;
	struct tankinputmodel input;
	struct processmodel process;
	int invid=0;
	int pumpinid=0;
	int process1=0;
	int process2=0;
	int process3=0;
	float processvalue1=0;
	float processvalue2=0;
	float processvalue3=0;

	time_t currtime;
	char charTime[10];
	unsigned int decAlarmTime;

	char *malarm[20];
	malarm[0]="Tank empty";
	malarm[1]="Tank normal level";
	malarm[3]="Tank full";
	malarm[11]="Tank almost full";
	malarm[12]="Tank leaking";

	if (0){
		printf("tank id ... %d\n", id);
	}

	tank = tanktable[id];
	input = tankinputtable[id];
	
	process1 = input.processinput1;
	process = processtable[process1];
	processvalue1=process.value;

	pumpinid = input.pumpinid;
	pump = pumptable[pumpinid];

	
	invid = input.tankid;
	tank.volume = tank.volume + pump.flow - processvalue1;

	if (tank.volume < tank.emptylimit) tank.volume = tank.emptylimit ;
	if (tank.volume > tank.volumemax) tank.volume = tank.volumemax ;

	if (0){
		printf("Tank\n");
	}

	if (tank.volume > tank.fullwarninglimit ){
		tank.tankstatus = 11;
	}
	if (0){
		printf("tank\n");
		printf("tank.tankid             ");
		printf("%5d\n", tank.tankid);			// stat: tank id
		printf("tank.tankstatus         ");
		printf("%5d\n", tank.tankstatus);		// out:  see below
		printf("tank.volume              ");
		printf("%5.1f\n", tank.volume);			// out:  actual current
		printf("tank.temperature_act     ");
		printf("%5.1f\n", tank.temperature_act);	// stat: tank temperature
	}

	printouttank(id);	// print out the info

	if (tank.tankstatus != tank.tankstatusprev){
		// print out alarm time
		time(&currtime);  // tarvitaan. hakee varmaan ajan tohon currtimeen
		strftime(charTime,sizeof(charTime)-1,"%H%M%S",localtime(&currtime));
		decAlarmTime = atoi(charTime);			// alarm on time
		printf(" %6d", decAlarmTime);
		// print out alarm text
		printf (" %s",malarm[tank.tankstatus]);
	}

	tank.tankstatusprev = tank.tankstatus;
	tanktable[id] = tank;
}

void simulate(int step){
	calculateinverter(1);
	// printf("\n                           ");
	printf("\n");
	calculatemotor(1);
	printf("\n");
	calculatepump(1);
	printf("\n");
	calculatetank(1);
	printf("\n");
	
}

void initprocess(){
	// muuta lukemaan filusta
	printf("Init begins\n");
	struct processmodel process;

	process.processid = 0;
	process.value=18;
	strcpy(process.text,"Air temperature");
	updateprocesstable(0, process);

	process.processid = 1;
	process.value=15;
	strcpy(process.text,"Pool temperature");
	updateprocesstable(1, process);

	process.processid = 2;
	process.value=15000;
	strcpy(process.text,"Pool liters");
	updateprocesstable(2, process);

	process.processid = 3;
	process.value=0;
	strcpy(process.text,"First pipe flow");
	updateprocesstable(3, process);

	process.processid = 4;
	process.value=71;
	strcpy(process.text,"Tank liters");
	updateprocesstable(4, process);

	process.processid = 5;
	process.value=90;  // small tank flow out
	strcpy(process.text,"Flow out");
	updateprocesstable(5, process);

	// xxx init moottori1 static values
	struct motormodel moottori1;
	moottori1.motorid=1;			// stat: motor id
	moottori1.motorstatus = 1;		// out:  motor status as default ok
	moottori1.motorstatusprev = 0;		// out:  motor status as default ok
	moottori1.frequency=0;			// stat: Hz
	moottori1.revspermin_act=0;		// calc: actual rpm
	moottori1.revspermin_nom=1000;		// stat: nominal revs per min (1000)
	moottori1.nomvoltage=400;		// stat: nominal voltage 400V
	moottori1.voltage=0;			// stat: voltage 400V
	moottori1.current=0;			// stat: current A
	moottori1.current_max=10.1;		// stat: current max 10.1A
	moottori1.cosfii=0.91;			// stat: cos fii
	moottori1.temperature_act=10;		// stat: motor temperature actual
	moottori1.temperature_al=70;		// stat: motor temperature max (alarm)
	moottori1.temperature_bd=90;		// stat: motor temperature max (break down)
	moottori1.temperaturerampup=0.5;	// stat: temperature delta C/s
	moottori1.rampup=2;			// stat: speed (rpm) up time s
	moottori1.rampdown=3;			// stat: speed (rpm) down time s
	moottori1.torque=0;			// calc: 
	moottori1.power=0;			// calc: 
	moottori1.runningtime=0;		// calc: 
	moottori1.runningtimelimit=600;		// stat: motor running time limit for calc 600 s
	moottori1.standstilltime=100;		// calc: 

	updatemotortable(0,moottori1);
	updatemotortable(1,moottori1);

	if (0){
		//moottori1 = motortable[1];
		printf("%6.0f %4.3f %6.1f %6.2f %6.1f %6.1f %5.1f %2d %#7.2x %d\n",
					moottori1.revspermin_act,
					moottori1.current,
					moottori1.temperature_act,
					moottori1.power,
					moottori1.voltage,
					moottori1.torque,
					moottori1.runningtime,
					moottori1.motorstatus,
					moottori1.motorstatus,
					moottori1.motorid);
	}


	struct motorinputmodel motorinputconnection;
	motorinputconnection.motorid=1;
	motorinputconnection.invid=1;
	motorinputconnection.frequency=0;
	motorinputconnection.voltage=0;
	motorinputconnection.torque=0;
	motorinputconnection.processinput1=4;
	motorinputconnection.processinput1=0;
	motorinputconnection.processinput1=0;
	updatemotorinputtable(0,motorinputconnection);
	updatemotorinputtable(1,motorinputconnection);

	struct invertermodel inverter1;
	inverter1.inverterid=1;			// stat: motor id
	inverter1.inverterstatus=1;		// as default inverter ok
	inverter1.inverterstatusprev=1;		// as default inverter ok
	inverter1.nomvoltage=400;		// stat: voltage 400V
	inverter1.current_max=10;		// stat: current max 10A
	inverter1.rampup=5;			// stat: speed up time 1s
	inverter1.rampdown=5;			// stat: speed down time 1s
	inverter1.setpoint=0;			// input:  0-100%
	inverter1.frequency=0;			// out:  frequency
	inverter1.voltage=0;			// out:  voltage V
	inverter1.voltagein=0;			// in:   voltage V
	inverter1.current=0;			// out:  actual current
	inverter1.power=0;			// calc: motor load kW

	updateinvertertable(0, inverter1);
	updateinvertertable(1, inverter1);

	if (0){
		// inverter1 = invertertable[1];
		printf("%d %6.0f %4.3f %6.1f %6.2f %6.1f %6.1f %5.1f %5.1f %5.1f %#7.2x\n",
					inverter1.inverterid,
					inverter1.nomvoltage,
					inverter1.current_max,
					inverter1.rampup,
					inverter1.rampdown,
					inverter1.setpoint,
					inverter1.frequency,
					inverter1.voltage,
					inverter1.current,
					inverter1.power,
					inverter1.inverterstatus);
	}

	struct inverterinputmodel inverterinputconnection;
	inverterinputconnection.inverterid=1;
	inverterinputconnection.frequency=50;
	inverterinputconnection.voltage=400;
	inverterinputconnection.setpoint=0;
	inverterinputconnection.processinput1=4; // upper tank level
	inverterinputconnection.processinput2=2; // big tank level
	inverterinputconnection.processinput3=2;
	updateinverterinputtable(0, inverterinputconnection);
	updateinverterinputtable(1, inverterinputconnection);

	struct pumpmodel pump;
	pump.pumpid=1;			// stat: motor id
	pump.pumpstatus=0;		// out:  see below
	pump.pumpstatusprev=0;		// out:  see below
	pump.revspermin_act=0;		// input:  0-1000
	pump.revspermin_nom=1000;	// stat:   0-1000
	pump.torque=0;			// out:  
	pump.flow=0;			// out:   flow 0-1000liters/s
	pump.flowmax=150;		// out:   flow 0-1000liters/s
	pump.runningtime=0;		// calc:  
	pump.temperature_act=14;	// calc:  
	pump.temperature_al=110;	// calc:  
	pump.temperature_bd=140;	// calc:  

	updatepumptable(0,pump);
	updatepumptable(1,pump);

	struct pumpinputmodel pumpinputconnection;
	pumpinputconnection.pumpid=1;		// stat:  pump id
	pumpinputconnection.revspermin_act=0;	// input:  0-1000
	pumpinputconnection.processinput1=0;
	pumpinputconnection.processinput2=0;
	pumpinputconnection.processinput3=0;
	updatepumpinputtable(0,pumpinputconnection);
	updatepumpinputtable(1,pumpinputconnection);

	struct tankmodel tank;
	tank.tankid=1;			// stat:  motor id
	tank.tankstatus=0;		// out:   see below
	tank.tankstatusprev=0;		// calc:  see below
	tank.flowin=0;			// input:  0-10
	tank.flowinmax=10;		// stat:     10
	tank.flowout=0;			// input:  0-10
	tank.flowoutmax=10;		// stat:     10
	tank.volume=0;			// calc:  
	tank.volumemax=15010;		// stat:  15000
	tank.temperature_act=14;	// calc: 
	tank.temperature_al=55;		// stat:
	tank.emptylimit=10;		// stat:
	tank.normallimit=100;		// stat:
	tank.fullwarninglimit=14000;	// stat:
	tank.leakinglimit=14500;	// stat:

	updatetanktable(0,tank);
	updatetanktable(1,tank);

	struct tankinputmodel tankinputconnection;
	tankinputconnection.tankid=1;		// stat: tank id
	tankinputconnection.pumpinid=1;		// out:  see below
	tankinputconnection.pumpoutid=0;	// out:  see below
	tankinputconnection.processinput1=5;	// stat:  
	tankinputconnection.processinput2=5;	// stat:  
	tankinputconnection.processinput3=5;	// stat:  

	updatetankinputtable(0,tankinputconnection);
	updatetankinputtable(1,tankinputconnection);

	printf("Init done\n");

}


int main(int argc, char *argv[]){
    if (argc < 2 ) {
            printf ("Enter loops\n");
            exit(1);
    }
    int imax = atoi(argv[1]);

    fd_set master;    // master file descriptor list
    fd_set read_fds;  // temp   file descriptor list for select()
    int fdmax;        // maximum file descriptor number

    int listener;     // listening socket descriptor
    int newfd;        // newly accept()ed socket descriptor
    struct sockaddr_storage remoteaddr; // client address
    socklen_t addrlen;

    char buf[256] = "0";    // buffer for client data
    int nbytes;
    char message[24];

    char remoteIP[INET6_ADDRSTRLEN];

    int yes=1;  // for setsockopt() SO_REUSEADDR, below
    int o;      // outmost loop
    int i;
    int j;
    int rv;
    int sendloop = 0;

    // timeout for select
    struct timeval tv;
    tv.tv_sec = 2;
    tv.tv_usec = 500000;

    struct addrinfo hints, *ai, *p;

    FD_ZERO(&master);    // clear the master and temp sets
    FD_ZERO(&read_fds);

    // xxx motorpart init
    time_t currtime;
    // struct timeval tv;
    // xxx this is not yet used
    // OR this is defined in getch()
    // but use it for select time out
    // when you add keypress getch()

    char c=0;
    //c = getch();
    struct timespec nanos;
    unsigned int decDate;
    unsigned int decTime;
    unsigned int decAlarmTime;
    unsigned int decSec;
    unsigned int decTimeAtStart;
    unsigned int decTimeAtEnd;
    unsigned int decTimeDelta = 1000;
    char charDate[10];
    char charTime[10];
    char charSec[10];
    // unsigned int millisec;
    clock_t millisec;

    // memorize ticks at the beginning
    if ((millisec = clock()) == -1)
        printf ("error by clock\n");

    int iii; // xxx change iii loop index
    int sleeptime = 1; // actual delay period default 5 sec
    unsigned int sleeptimemax = 10; // max delay period default 10 sec
    unsigned int testi = 5; // for %u print

    // init nanosleep
    int long nanosleeptime = 400000000L;   // 0,5sec
    int long nanosleeptimemax = 900000000L;
    nanos.tv_sec =0;
    nanos.tv_nsec=5000000L;

    time(&currtime);  // get time to currtime
    strftime(charDate,sizeof(charDate)-1,"%Y%m%d",localtime(&currtime));
    strftime(charTime,sizeof(charTime)-1,"%H%M%S",localtime(&currtime));

    decDate = atoi(charDate);	// date here
    decTime = atoi(charTime);	// time here

    printf("%d\n", decDate);
    printf("%d\n", decTime);
    printf("%u\n",(unsigned int)millisec);	// ticks at start
    printf("%ld\n",(long int)clock());	// ticks at end fff

    // strtok token init
    char *sendcopy;
    char *token;
    const char tokenseparator[2] = ";";
    char toslicing[80] = "b";
    char *receivedtelegram = "aaaa;12;dddd;21";
    char *telegramdevice = "0";
    char *telegramdevicenum = "0";
    char *telegramcommand = "0";
    char *telegramvalue = "0";
    int   telegramvalueint = 0;
    int   telegramdevicenumint = 0;
    char asctonum[3];
    char *issetpointcommand = "setpoint";

    time(&currtime);  // tarvitaan. hakee varmaan ajan tohon currtimeen
    strftime(charTime,sizeof(charTime)-1,"%H%M%S",localtime(&currtime));
    decTime = atoi(charTime);	// start time here
    decTimeAtEnd = decTime;		// value for comparison

    // networkpart starts.  Get us a socket and bind it
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE;
    if ((rv = getaddrinfo(NULL, PORT, &hints, &ai)) != 0) {
        fprintf(stderr, "selectserver: %s\n", gai_strerror(rv));
        exit(1);
    }

    for(p = ai; p != NULL; p = p->ai_next) {
        listener = socket(p->ai_family, p->ai_socktype, p->ai_protocol);
        if (listener < 0) { 
            continue;
        }
        
        // lose the pesky "address already in use" error message
        setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int));

        if (bind(listener, p->ai_addr, p->ai_addrlen) < 0) {
            close(listener);
            continue;
        }
        break;
    }

    // if we got here, it means we didn't get bound
    if (p == NULL) {
        fprintf(stderr, "selectserver: failed to bind\n");
        exit(2);
    }
    printf("listener .... %d\n", listener);

    freeaddrinfo(ai); // all done with this

    // listen
    if (listen(listener, 10) == -1) {
        perror("listen");
        exit(3);
    }

    // add stdin (keypress detection) to the master set
    // add socket listener to the master set
    FD_SET(0, &master);
    FD_SET(listener, &master);

    // keep track of the biggest file descriptor
    fdmax = listener; // so far, it's this one

    // xxx init motor values
    struct motormodel moottori1;
    struct motorinputmodel motorinputconnection;
    struct invertermodel inverter1;
    struct inverterinputmodel inverterinputconnection;
    struct processmodel process;

    data_count_motor = 0;
    data_count_motor_input = 0;
    data_count_inverter = 0;
    data_count_inverter_input = 0;

    data_count_pump = 0;
    data_count_pump_input = 0;
    data_count_tank = 0;
    data_count_tank_input = 0;
    data_count_process = 0;

    data_size_motor = INITIAL_SIZE;
    data_size_motor_input = INITIAL_SIZE;
    data_size_inverter = INITIAL_SIZE;
    data_size_inverter_input = INITIAL_SIZE;
    data_size_pump = INITIAL_SIZE;
    data_size_pump_input = INITIAL_SIZE;
    data_size_tank = INITIAL_SIZE;
    data_size_tank_input = INITIAL_SIZE;
    data_size_process = INITIAL_SIZE;

    if (( motortable = malloc(data_size_motor * sizeof(struct motormodel))) == NULL){
            printf("002 Uuups. Out of memory error\n");
    }

    if (( motorinputtable = malloc(data_size_motor_input * sizeof(struct motorinputmodel))) == NULL){
            printf("002 Uuups. Out of memory error\n");
    }

    if (( invertertable = malloc(data_size_inverter * sizeof(struct invertermodel))) == NULL){
            printf("002 Uuups. Out of memory error\n");
    }
    if (( inverterinputtable = malloc(data_size_inverter_input * sizeof(struct inverterinputmodel))) == NULL){
            printf("002 Uuups. Out of memory error\n");
    }

    if (( pumptable = malloc(data_size_pump * sizeof(struct pumpmodel))) == NULL){
            printf("002 Uuups. Out of memory error\n");
    }
    if (( pumpinputtable = malloc(data_size_pump_input * sizeof(struct pumpinputmodel))) == NULL){
            printf("002 Uuups. Out of memory error\n");
    }

    if (( tanktable = malloc(data_size_tank * sizeof(struct tankmodel))) == NULL){
            printf("002 Uuups. Out of memory error\n");
    }

    if (( tankinputtable = malloc(data_size_tank_input * sizeof(struct tankinputmodel))) == NULL){
            printf("002 Uuups. Out of memory error\n");
    }

    if (( processtable = malloc(data_size_process * sizeof(struct processmodel))) == NULL){
            printf("002 Uuups. Out of memory error\n");
    }

    // float frequencyfrominvtomotor = 0;	// 0-50Hz
    float torquetomotor = 1.93;		// Nm
    int   torqueup = 0;			// dynamic torque up 0= down
    float voltagetomotor = 0;		// 0-400V

    initprocess();

    time(&currtime);  // tarvitaan. hakee varmaan ajan tohon currtimeen
    strftime(charDate,sizeof(charDate)-1,"%Y%m%d",localtime(&currtime));
    strftime(charTime,sizeof(charTime)-1,"%H%M%S",localtime(&currtime));

    decDate = atoi(charDate);		// date here
    decTime = atoi(charTime);		// time here

    printf("%d\n", decDate);
    printf("%d\n", decTime);
    printf("%u\n",(unsigned int) millisec); // ticks at start xxx korjaa cast
    printf("%u\n",(unsigned int)clock());	// ticks at end

    printinforow();				// print first inforow

    time(&currtime);  // tarvitaan. hakee varmaan ajan tohon currtimeen
    strftime(charTime,sizeof(charTime)-1,"%H%M%S",localtime(&currtime));
    decTime = atoi(charTime);		// start time here
    decTimeAtEnd = decTime;			// value for comparison

    // main loop zzz ok tämä ei toimi kun se jää tuohon seuraavaan looppiin.
    // ei toiminut, koska siinä oli typerästi sama i laskurissa.
    // poista
    //for ( i=0; i<imax ; i++){
    for ( o=0; o<imax ; o++){
          read_fds = master; // copy it
          tv.tv_sec = 2;
          tv.tv_usec = 500000;

            if (select(fdmax+1, &read_fds, NULL, NULL, &tv) == -1) {
              perror("select");
              exit(4);
            }

        // run through the existing connections looking for data to read
        for( i = 0; i <= fdmax; i++) {
            if (FD_ISSET(i, &read_fds)) { // we got one!!
                if (i == 0) {
                    printf("Local control by keypress\n");
                        if (read(STDIN_FILENO, &c, 1) == -1)
                        {
                            perror("getch:read() failed!");
                        }
                        else
                        {
                            printf ("keypressed was ... %c\n", c);
                            if ( c == 'q' || c == 'Q' ){
                                free(motortable);
                                free(motorinputtable);
                                free(invertertable);
                                free(inverterinputtable);
                                free(pumptable);
                                free(pumpinputtable);
                                free(tanktable);
                                free(tankinputtable);
                                free(processtable);
                                close(i); // bye!
                                return 0;

                            }
                            continue;
                            
                        }
                }
                if (i == listener) {
                    // handle new connections
                    addrlen = sizeof remoteaddr;
                    newfd = accept(listener,
                        (struct sockaddr *)&remoteaddr,
                        &addrlen);

                    if (newfd == -1) {
                        perror("accept");
                    } else {
                        FD_SET(newfd, &master); // add to master set
                        if (newfd > fdmax) {    // keep track of the max
                            fdmax = newfd;
                        }
                        printf("selectserver: new connection from %s on "
                            "socket %d\n",
                            inet_ntop(remoteaddr.ss_family,
                                get_in_addr((struct sockaddr*)&remoteaddr),
                                remoteIP, INET6_ADDRSTRLEN),
                            newfd);
                    }
                } else {
                    // handle data from a client
                    if ((nbytes = recv(i, buf, sizeof buf, 0)) <= 0) {
                        // got error or connection closed by client
                        if (nbytes == 0) {
                            // connection closed
                            printf("selectserver: socket %d hung up\n", i);
                        } else {
                            perror("recv");
                        }
                        close(i); // bye!
                        FD_CLR(i, &master); // remove from master set
                    } else {
		      printf ("telegram .... %s\n", buf);
		      printf ("bytes ....... %d\n", nbytes);
                        // we got some data from a client
                        for(j = 0; j <= fdmax; j++) {
			 // printf ("j ........... %d\n", j);
                            // send to everyone!
                            if (FD_ISSET(j, &master)) {
                                // except the listener and ourselves
				if (j != listener) {
                                  if (send(j, buf, nbytes, 0) == -1) {
                                        perror("send");
                                    }
                                }
                            }
			}
                    }
                } // END handle data from client
            } // END got new incoming connection
        } // END looping through file descriptors
	// printf ("telegram buf .............. %s\n", buf);

	(*toslicing) = '\0';
        telegramdevice = NULL;
        telegramdevicenum = NULL;
        telegramcommand = NULL;
        telegramvalue = NULL;

       	strcpy(toslicing,buf);
	//tai näillä saa nollattua buffin kierroksen jälkeen
	//snprintf(toslicing, sizeof(buf), "%s", buf);
        //buf[0] = '\0';
	
	telegramdevice = strtok(toslicing, tokenseparator);
	telegramdevicenum = strtok(NULL, tokenseparator);
	// printf ("%d\n", atoi(telegramdevicenum));

	telegramcommand = strtok(NULL, tokenseparator);
	telegramvalue = strtok(NULL, tokenseparator);
	
	// strcpy(asctonum, telegramvalue);
	snprintf(asctonum, 3, "%s" , telegramdevicenum);
	telegramdevicenumint = atoi(asctonum);
	
	snprintf(asctonum, 3, "%s" , telegramvalue);
	telegramvalueint = atoi(asctonum);

	printf ("\n");
	printf ("telegram buf .............. %s\n", buf);
	printf ("telegram device ........... %s\n", telegramdevice);
	printf ("telegram device num ....... %s\n", telegramdevicenum);
	printf ("telegram device num int ... %d\n", telegramdevicenumint);

	printf ("telegram command .......... %s\n", telegramcommand);
	printf ("telegram value ............ %s\n", telegramvalue);
	printf ("telegram value num int .... %d\n", telegramvalueint);

	if ( telegramcommand != NULL ) {
		if ( strcmp(telegramcommand, "inc" ) == 0 ) addinvrevs(telegramdevicenumint);
		if ( strcmp(telegramcommand, "dec" ) == 0 ) decinvrevs(telegramdevicenumint);
	}

	if ( telegramcommand != NULL ) {
		if ( strcmp(telegramcommand, "setpoint" ) == 0 ) setinvsetpoint(telegramdevicenumint,telegramvalueint);
	}
	// zzz jatka lisää nämä
	// if ( telegramvalueint == 4 ) setpoint++;
	// if ( telegramvalueint == 5 ) setpoint--;
	// if ( telegramvalueint == 14 ) { torquetomotor += 0.1;}
	// if ( telegramvalueint == 15 ) { torquetomotor -= 0.1;}

	simulate(1);

    } // END for(;;)--and you thought it would never end!
    return (0);
}

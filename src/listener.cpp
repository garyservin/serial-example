/////////////////////
// 헤더 파일 선언 구간 //
/////////////////////
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud.h"





/////////////////
// 변수 선언 구간 //
/////////////////
char incomingData[320] = "";      // Rosserial을 통해 받는 통신 값을 저장하는 버퍼
int start_flag = 0;             // incomingData의 값들을 받을지 결정하는 플래그 변수
int i = 0;                      // incomingData의 인덱스 변수
int k = 0;							// n번째 센서 값 버퍼(구조체)에 대한 인덱스 변수
int l = 0;							// n번째 센서 값의 x, y, z값에 대한 인덱스 변수
int kk = 0;							// 센서 9개에 대한 인덱스 변수

char sensor_num_char = 0;                     // 값을 확인할 센서를 선택하는 변수
int sensor_num_int = 0;

int mode = 0;                    // 센서 값 분리 작업 중 단계를 거치기 위한 플래그 변수

char header[3] = { 0, };         // 패킷의 헤더를 분리 후 보관할 배열 변수
char checkSum_arr[10];			 // 패킷에서 넘겨받은 checksum 배열 값 저장 변수
int checkSum_here = 0;           // checkSum_arr에서 정수형으로 변환한 값을 저장하는 변수
int checkSum = 0;                // 패킷 분리 작업 후 실제로 다시 checksum을 계산한 값을 저장하는 변수
int count_num = 0;

char large_cap = 'A';		     // n번째 센서 값이 시작되는 것을 비교/확인하기 위한 변수
char small_cap = 'a';			 // n번째 센서 값이 끝나는 것을 비교/확인하기 위한 변수

// 홀 센서 값을 저장하는 구조체 변수
struct hallsensor_struct {
	char full_val[29] = "";			// 패킷에서 n번째 센서값을 뭉텅이로 먼저 저장하는 임시 버퍼
	char x_val[10] = "";				// n번째 센서 값의 x축 값
	char y_val[10] = "";				// n번째 센서 값의 y축 값
	char z_val[10] = "";				// n번째 센서 값의 z축 값 
										// (음수 부호와 null 까지 고려하여 크기를 6으로 설정)

	int x_val_int = 0;
	int y_val_int = 0;
	int z_val_int = 0;				// x, y, z_val에서 정수형으로 변환 후 저장할 변수 선언
									// checksum을 구해야 하므로 먼저 정수형을 구해야 함

	double x_val_real = 0;
	double y_val_real = 0;
	double z_val_real = 0;			// 정수형으로 변환 후, 손실된 소수점 복구한 최종 센서 값 저장

}hall_[9];                          // 센서가 9개이므로 구조체 변수 9세트 할당

int timerr = 0;
int buffer_size = 0;				// 버퍼로 받는 문자열의 길이 값을 저장하는 변수

int print_check_mode = 0;			// 테스트 출력 설정 모드












/////////////////
// 함수 선언 구간 //
/////////////////
void working_func();        // 본격적으로 센서 값의 분리 작업을 진행하는 함수
void chatterCallback(const std_msgs::String::ConstPtr& msg);    // Rosserial을 통해 subscribe하는 값을 적절히 받아서 완전한 형태의 패킷으로 정리 & 배열로 다시 저장하는 함수
void modeCallback(const std_msgs::String::ConstPtr& msg);       // 몇 번 센서 값을 출력해서 볼지 결정해주는 변수 값 메세지 입력받는 함수

void reset_func();                                              // 패킷 분리작업 때 사용한 변수들 초기화하는 함수
void hall_buffer_func();                                        // n번째 센서 값 별로 분리 후 저장해주는 함수
void seperate_axis_val();                                       // n번째 센서 값을 x, y, z축별로 세부 분리 후 실수형으로 따로 저장해주는 함수
void turn2real();                                               // 모든 센서 값을 정수형에서 실수형으로 최종 변환해주는 함수
void allPrint();												// 모든 센서 값을 출력해주는 함수
int	custom_atoi(const char *str);                               // atoi함수 구현










///////////
// 메인문 //
//////////
int main(int argc, char **argv)
{   

  ros::init(argc, argv, "listener");
  ros::NodeHandle serial;

  // 먼저 여기를 통해 통신 데이터 수신을 받는다. -> chatterCallback 함수 호출
  // ros::Subscriber sensor_mode = serial.subscribe("sensor", 1000, modeCallback);
  ros::Subscriber sub = serial.subscribe("read", 1000, chatterCallback);
  ros::Publisher pub_all = serial.advertise<sensor_msgs::PointCloud>("local_sys", 100);

	while(ros::ok()){

		ros::spinOnce();

		sensor_msgs::PointCloud all_sensor;			// 메세지 구조체 선언
		all_sensor.points.resize(9);				// Points 개수는 총 9개

		for(kk=0; kk < 9; kk++){					// 반복문을 통해 센서 9개 값 메세지에 저장
			all_sensor.points[kk].x = hall_[kk].x_val_real;
			all_sensor.points[kk].y = hall_[kk].y_val_real;
			all_sensor.points[kk].z = hall_[kk].z_val_real;
		}
		
		pub_all.publish(all_sensor);				// 최종적으로 publish

	}

  

  return 0;
}










/////////////////
// 함수 정의 구간 //
/////////////////

/*
// 몇 번 센서 값을 출력해서 볼지 결정해주는 변수 값 메세지 입력받는 함수
void modeCallback(const std_msgs::String::ConstPtr& msg)
{
  sensor_num_char = msg->data[0];   // sensor_select node에서 해당 값을 입력받음
  sensor_num_int = sensor_num_char - 48;            // 사용하기 편하게 정수형으로 바꿔서 저장
}
*/


// Rosserial을 통해 subscribe하는 값을 적절히 받아서 완전한 형태의 패킷으로 정리 & 배열로 다시 저장하는 함수
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	// 32개씩 10개의 문자열이 들어오므로, Z가 검출될 때를 기준으로 10번 버퍼에 담으면 된다.
	if(start_flag == 0)
    	if(msg->data[0] == 'Z') start_flag = 1;      // 1. 수신받은 문자열 첫 문자가 Z로 시작하면 start_flag 1로 
    

    // 2. start_flag 값이 1이면 
	if(start_flag == 1) {
        for(int k=0; k < 32; k++)
			incomingData[i++] = msg->data[k];   // 3. 들어오는 문자열 이어서 버퍼에 저장
		
		count_num++;		// 분할 패킷을 총 몇 번 받았는지 카운트

		if(count_num == 10) start_flag = 2;		// 분할 패킷을 총 10번 받았으면 완성된 것이므로 다음 작업으로
	}


	if(start_flag == 2){
		i = 3;                                  // 6. 인덱스 초기화
    	start_flag = 0;                         // 7. 버퍼에 통신 값 저장을 알리는 플래그 변수도 0으로 
		count_num = 0;
    	mode = 1;                               // 8. 센서 값 분리 모드 On
    	working_func();                         // 9. 센서 값 분리하는 함수 호출 -> 본격적인 작업 시작
	}
	
}



// 본격적으로 센서 값의 분리 작업을 진행하는 함수
void working_func(){

    if (mode == 1) {
		printf("%s\n", incomingData);       // 문자열로 출력
		// 헤더 분리
		//for (kk = 0; kk < 3; kk++) header[kk] = incomingData[i++];

		// 센서마다 값 분리
		for (kk = 0; kk < 9; kk++) {
			hall_buffer_func();
			if (print_check_mode == 1) printf("%s\n", hall_[kk].full_val);
		}
	
    	// checksum 값 분리
		for (k = 0; k < 9; k++) {	// checksum 값 10개만 딱 받기
			checkSum_arr[k] = incomingData[i++];	// 차례로 checksum 변수에 저장
		}
		k = 0;						// 다른 곳에서 사용될 것을 감안하여 초기화를 해 준다

		mode++;                     // 다음 작업으로

        }


    // mode = 2일때: 각 센서값을 x, y, z값으로 분해 후 정수형으로 변환
		if (mode == 2) {

			// 축 별로 값 분리 후 정수형으로 변환
			// if (print_check_mode == 1) printf("------------------------------------\n");
			for (kk = 0; kk < 9; kk++) {
				seperate_axis_val();
				//if (print_check_mode == 1) printf("%d %d %d\n", hall_[kk].x_val_int, hall_[kk].y_val_int, hall_[kk].z_val_int);
			
			}
			//printf("process complete4\n");

			// checksum 정수형으로 변환
			int sign = 0;
			if (checkSum_arr[0] == '1') { checkSum_arr[0] = '0'; sign = -1; }
			else sign = 1;
			checkSum = custom_atoi(checkSum_arr) * sign;

			// 지금까지 분리했던 모든 센서 값들 전부 더하기
			for (kk = 0; kk < 9; kk++)
				checkSum_here += (hall_[kk].x_val_int) + (hall_[kk].y_val_int) + (hall_[kk].z_val_int);


			//if (print_check_mode == 1) printf("%d, %d\n", checkSum, checkSum_here);

			// checksum 비교
			if (checkSum == checkSum_here) {
				for (kk = 0; kk < 9; kk++) turn2real();
				//printf("Packet Comm Success!!\n");
				//printf("x: %f | y: %f | z: %f | count: %d %d\n", hall_[sensor_num_int].x_val_real, hall_[sensor_num_int].y_val_real, hall_[sensor_num_int].z_val_real, timerr++, sensor_num_int);
				allPrint();			// 센서 값 출력
			}
			else {
				//printf("Try again...\n");
				//mode = 2;
			}	// 그러나 일치하지 않으면 다시 처음부터

			reset_func();	// 한 번 통신이 끝나면 지금까지 사용했던 변수들 전부 초기화
		}
    

}




// n번째 센서 값 별로 분리 후 저장해주는 함수
void hall_buffer_func() {

	for (kk = 0; kk < 9; kk++) {
		for (int iii=0; iii < 29; iii++) 					// 딱 29개의 문자를 저장 
			hall_[kk].full_val[iii] = incomingData[i++];	// 센서값 통째로 버퍼에 저장
		
		i = i+2;	// 중간중간 껴 있는 알파벳 무시하고 바로 다음 센서 값으로
	}
	i--;			// 센서 값 이후 index를 Checksum에 위치시키기 위한 조절
}



// n번째 센서 값을 x, y, z축별로 세부 분리 후 정수형으로 따로 저장해주는 함수
void seperate_axis_val(){

	int sign = 0;		// 센서 값의 부호를 결정하기 위한 부호 변수(정수)
	k = 0;

	for(kk = 0; kk <9; kk++){		// n번째 센서 선택

		for(l = 0; l < 9; l++) hall_[kk].x_val[l] = hall_[kk].full_val[k++];	// n번째 센서 값의 x값 추출	
		if (print_check_mode == 1) std::cout << hall_[kk].x_val;
		if (print_check_mode == 1) std::cout << "  ";
		if (hall_[kk].x_val[0] == '1') { hall_[kk].x_val[0] = '0'; sign = -1; }	// 패킷 맨 앞자리가 1이면 음수이므로, 0으로 바꾼 후 음수 연산자 설정
		else sign = 1;				// 그런데 0으로 시작하면 양수이므로, 양수 연산자로 설정
		hall_[kk].x_val_int = custom_atoi(hall_[kk].x_val) * sign;	// 추출한 값을 정수형으로 변환 + 부호 복원

		k++;		// 콤마 패스

		for(l = 0; l < 9; l++) hall_[kk].y_val[l] = hall_[kk].full_val[k++];	// n번째 센서 값의 y값 추출	
		if (print_check_mode == 1) std::cout << hall_[kk].y_val;
		if (print_check_mode == 1) std::cout << "  ";
		if (hall_[kk].y_val[0] == '1') { hall_[kk].y_val[0] = '0'; sign = -1; }
		else sign = 1;
		hall_[kk].y_val_int = custom_atoi(hall_[kk].y_val) * sign;

		k++;		// 콤마 패스

		for(l = 0; l < 9; l++) hall_[kk].z_val[l] = hall_[kk].full_val[k++];	// n번째 센서 값의 z값 추출	
		if (print_check_mode == 1) std::cout << hall_[kk].z_val ;
		if (print_check_mode == 1) std::cout << "\n\n";
		if (hall_[kk].z_val[0] == '1') { hall_[kk].z_val[0] = '0'; sign = -1; }
		else sign = 1;
		hall_[kk].z_val_int = custom_atoi(hall_[kk].z_val) * sign;

		k = 0;
		l = 0;		// n+1번째 센서 값의 세부 값 추출을 위해 인덱스 변수 초기화

	}

}




// 모든 센서 값을 정수형에서 실수형으로 최종 변환해주는 함수
void turn2real() {
	for (kk = 0; kk < 9; kk++){
		hall_[kk].x_val_real = (double)hall_[kk].x_val_int / 100 * (-1);	// 왼손법칙 좌표계에서 오른손법칙 좌표계로 변환하기 위해 부호 변경
		hall_[kk].y_val_real = (double)hall_[kk].y_val_int / 100;			// 정수 변환 때는 checksum 계산 때문에 나중에 부호 변경을 해야 함
		hall_[kk].z_val_real = (double)hall_[kk].z_val_int / 100;
	}
}



// 모든 센서 값을 출력해주는 함수
void allPrint() {
	// printf("%d\n\n", timerr++);
	for (kk = 0; kk < 9; kk++)
		printf("%dth sensor   x: %f | y: %f | z: %f | \n", kk+1, hall_[kk].x_val_real, hall_[kk].y_val_real, hall_[kk].z_val_real);
	printf("\n\n");

}								



// 패킷 분리작업 때 사용한 변수들 초기화하는 함수
void reset_func() {
	mode = 0;
	//for (kk = 0; kk < 9; kk++) hall_[kk] = { 0, };
	//for (kk = 0; kk < 4096; kk++) incomingData[kk] = { 0 };
	//for (kk = 0; kk < 10; kk++) checkSum_arr[kk] = 0;
	for (kk = 0; kk < 3; kk++) header[kk] = 0;
	i = 0;
	k = 0;
	l = 0;
	kk = 0;
	checkSum = 0;
	checkSum_here = 0;


	//large_cap = 'A';
	//small_cap = 'a';

}



// atoi 함수 구현
int	custom_atoi(const char *str)
{
	long long int	sign;
	long long int	value;

	sign = 1;
	value = 0;
	while (*str == '\t' || *str == '\n' || *str == '\v' \
			|| *str == '\f' || *str == '\r' || *str == ' ')
		str++;
	if (*str == '+' || *str == '-')
	{
		if (*str == '-')
			sign *= -1;
		str++;
	}
	while (*str >= 48 && *str <= 57)
	{
		value *= 10;
		value += *str - 48;
		str++;
	}
	return (value * sign);
}
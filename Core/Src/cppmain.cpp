#include "main.h"

#include <cmath>

#include "tutrc_harurobo_lib/uart.hpp"
#include "tutrc_harurobo_lib/can.hpp"
#include "tutrc_harurobo_lib/ps3.hpp"
#include "tutrc_harurobo_lib/c610.hpp"
#include "tutrc_harurobo_lib/timer.hpp"
#include "tutrc_harurobo_lib/encoder.hpp"
#include "tutrc_harurobo_lib/bno055.hpp"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern FDCAN_HandleTypeDef hfdcan1;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim3;

using namespace std;
using namespace tutrc_harurobo_lib;

int clamp(int x, int a, int b) {
	return min(max(x, a), b);
}

extern "C" void cppmain() {
	//HAL_Delay(1000);
	UART debug(&huart2);
	UART uart3(&huart3);
	CAN can(&hfdcan1);
	PS3 ps3(&huart1);
	C610 c610(can);
	Timer timer(&htim5);
	BNO055 bno(uart3);

	Encoder enc(&htim3, 2048, 0.01);

	timer.set_timer_callback([&]() {
		ps3.update();
		enc.update();

		double vx = ps3.get_axis(PS3::Axis::LEFT_X);
		double vy = -ps3.get_axis(PS3::Axis::LEFT_Y);

		double R = -bno.get_euler_x();
		//double R = 0;

		double v1 = 60 * (vx * cos(0 + R) + vy * sin(0 + R));
		double v2 = 60 * (vx * cos((M_PI * 2 / 3) + R) + vy * sin((M_PI * 2 / 3)+ R));
		double v3 = 60
				* (vx * cos((M_PI * 4 / 3) + R) + vy * sin((M_PI * 4 / 3) + R));

		if (ps3.get_key(PS3::Key::L1)) {
			v1 -= 40;
			v2 -= 40;
			v3 -= 40;
		}

		if (ps3.get_key(PS3::Key::R1)) {
			v1 += 40;
			v2 += 40;
			v3 += 40;
		}

		double K = 100;
		double e1 = v1 - c610.get_rps(C610::ID::ID1);
		int c1 = clamp(K * e1, -2500, 2500);

		double e2 = v2 - c610.get_rps(C610::ID::ID2);
		int c2 = clamp(K * e2, -2500, 2500);

		double e3 = v3 - c610.get_rps(C610::ID::ID3);
		int c3 = clamp(K * e3, -2500, 2500); //P制御

		c610.set_current(C610::ID::ID1, c1);
		c610.set_current(C610::ID::ID2, c2);
		c610.set_current(C610::ID::ID3, c3); //モーターを動かす

		c610.transmit(); //3つのモーターの指示

		//debug.printf("%f\t%f\t%f\t%f\t%f\r\n", vx, vy, v1, v2, v3);
	});

	while (true) {
		bno.update();
	}
}

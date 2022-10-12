/*
	Copyright 2019 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "app.h"
#include "ch.h"
#include "hal.h"

// Some useful includes
#include "mc_interface.h"
#include "utils_math.h"
#include "encoder/encoder.h"
#include "terminal.h"
#include "comm_can.h"
#include "hw.h"
#include "commands.h"
#include "timeout.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

// Threads
static THD_FUNCTION(my_thread, arg);
static THD_WORKING_AREA(my_thread_wa, 1024);

// Private functions
static void pwm_callback(void);
static void terminal_test(int argc, const char **argv);

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;

static float module_min = 1000.0;
static float module_max = 0.0;

static float sin_mean = 2.0;
static float cos_mean = 2.0;

static float sin_min = 1000.0;
static float sin_max = 0.0;

static float cos_min = 1000.0;
static float cos_max = 0.0;

static float mean_filter_value = 0.0001;

static bool autocal_enabled = false;

extern ENCSINCOS_config_t encoder_cfg_sincos;

// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void) {
	mc_interface_set_pwm_callback(pwm_callback);

	stop_now = false;
	chThdCreateStatic(my_thread_wa, sizeof(my_thread_wa),
			NORMALPRIO, my_thread, NULL);

	// Terminal commands for the VESC Tool terminal can be registered.
	terminal_register_command_callback(
			"custom_cmd",
			"Print the number d",
			"[d]",
			terminal_test);
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void) {
	mc_interface_set_pwm_callback(0);
	terminal_unregister_callback(terminal_test);

	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}
}

void app_custom_configure(app_configuration *conf) {
	(void)conf;
}

static THD_FUNCTION(my_thread, arg) {
	(void)arg;

	chRegSetThreadName("AppCustom");

	is_running = true;

	// Example of using the experiment plot
	//chThdSleepMilliseconds(8000);
	commands_init_plot("Sample", "Voltage");
	commands_plot_add_graph("SIN");
	commands_plot_add_graph("COS");
	commands_plot_add_graph("Vector");
	commands_plot_add_graph("ObsError");
	float samp = 0.0;

	for(;;) {
		// Check if it is time to stop.
		if (stop_now) {
			is_running = false;
			return;
		}

		timeout_reset(); // Reset timeout if everything is OK.

		// Run your logic here. A lot of functionality is available in mc_interface.h.

		/*float sin = (ENCODER_SIN_VOLTS - encoder_cfg_sincos.s_offset) * encoder_cfg_sincos.s_gain;
		float cos = (ENCODER_COS_VOLTS - encoder_cfg_sincos.c_offset) * encoder_cfg_sincos.c_gain;

		UTILS_LP_FAST(encoder_cfg_sinco.state.sin_filter, sin, encoder_cfg_sincos.filter_constant);
		UTILS_LP_FAST(encoder_cfg_sincos.state.cos_filter, cos, encoder_cfg_sincos.filter_constant);*/
		float sin = encoder_cfg_sincos.state.sin_filter;
		float cos = encoder_cfg_sincos.state.cos_filter;

		if (sin > sin_max) sin_max = sin;
		if (sin < sin_min) sin_min = sin;

		if (cos > cos_max) cos_max = cos;
		if (cos < cos_min) cos_min = cos;

		float module = SQ(sin) + SQ(cos);
		if (module < module_min) module_min = module;
		if (module > module_max) module_max = module;

		
		sin_mean = sin_mean * (1.0 - mean_filter_value) + mean_filter_value * sin;
		cos_mean = cos_mean * (1.0 - mean_filter_value) + mean_filter_value * cos;

		if (autocal_enabled){
			encoder_cfg_sincos.s_offset = sin_mean + encoder_cfg_sincos.s_offset;
			encoder_cfg_sincos.c_offset = cos_mean + encoder_cfg_sincos.c_offset;
		}

		commands_plot_set_graph(0);
		commands_send_plot_points(samp, sin);
		commands_plot_set_graph(1);
		commands_send_plot_points(samp, cos);
		commands_plot_set_graph(2);
		commands_send_plot_points(samp, module);
		commands_plot_set_graph(3);
		commands_send_plot_points(samp, utils_angle_difference(mcpwm_foc_get_phase_observer(), mcpwm_foc_get_phase_encoder()));

		
		samp++;
		//chThdSleepMilliseconds(10);
		//commands_printf(".");

		chThdSleepMilliseconds(10);
	}
}

static void pwm_callback(void) {
	// Called for every control iteration in interrupt context.
}

// Callback function for the terminal command with arguments.
static void terminal_test(int argc, const char **argv) {
	if (argc == 2) {
		float d = -1.0;
		sscanf(argv[1], "%f", &d);

		commands_printf("You have entered %f", d);

		// For example, read the ADC inputs on the COMM header.
		mean_filter_value = d;
	} else if (argc == 3) {
        if (strcmp(argv[1], "autocal") == 0){
            if (strcmp(argv[2], "1") == 0){
                autocal_enabled = true;
				commands_printf("autocal enabled");
            }else{
				autocal_enabled = false;
				commands_printf("autocal disabled");
			}
        }	
	} else {
	
		commands_printf("module_min: %.3f , module_max: %.3f, diff: %.3f", module_min, module_max, module_max-module_min);
		module_max = 0.0;
		module_min = 1000.0;

		commands_printf("sin_mean: %.4f , cos_mean: %.4f", sin_mean + encoder_cfg_sincos.s_offset, cos_mean + encoder_cfg_sincos.c_offset);

		commands_printf("sin ampl: %.4f , cos ampl: %.4f", sin_max - sin_min, cos_max - cos_min);
		sin_min = 1000.0;
		sin_max = 0.0;

		cos_min = 1000.0;
		cos_max = 0.0;
	
	}
}

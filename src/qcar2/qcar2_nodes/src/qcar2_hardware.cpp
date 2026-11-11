#include <chrono>
#include <functional>
#include <memory>
#include <map>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "quanser/hil.h"
#include "quanser/quanser_messages.h"
#include "quanser/quanser_types.h"
#include "quanser/quanser_led.h"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/string.hpp"
#include <sstream>
#include <algorithm>
#include <cmath>
#include <mutex>

#include "qcar2_interfaces/msg/motor_commands.hpp"
#include "qcar2_interfaces/msg/boolean_leds.hpp"

#define LED_STRIP_SIZE  33

using namespace std::chrono_literals;
using namespace std::placeholders;

class QCar2 : public rclcpp::Node
{
public:
    QCar2()
    : Node("qcar2")
    {
        t_int result;

        other_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = other_cb_group_;

        rclcpp::PublisherOptions pub_options;
        pub_options.callback_group = other_cb_group_;

        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};

        // Declare Gyro parameters
        param_desc.description = "Gyro configuration.";
        param_desc.additional_constraints = "This gyro parameter is for setting the QCar2 onboard gyroscrope parameters.";
        this->declare_parameter("gyro.fs", gyro_fs, param_desc);
        this->declare_parameter("gyro.rate", gyro_rate, param_desc);
        this->declare_parameter("gyro.bw", gyro_bw, param_desc);
        this->declare_parameter("gyro.ord", gyro_ord, param_desc);

        // Declare Accel parameters
        param_desc.description = "Accelerometer configuration.";
        param_desc.additional_constraints = "This accel parameter is for setting the QCar2 onboard accelerometer parameters.";
        this->declare_parameter("accel.fs", accel_fs, param_desc);
        this->declare_parameter("accel.rate", accel_rate, param_desc);
        this->declare_parameter("accel.bw", accel_bw, param_desc);
        this->declare_parameter("accel.ord", accel_ord, param_desc);

        param_desc.description = "IMU temperature bandwidth.";
        param_desc.additional_constraints = "This option sets the temperature sensor's filter bandwidth. Valid values range from 5 to 4000. The units are Hz.";
        this->declare_parameter("temp_bw", temp_bw, param_desc);

        param_desc.description = "Steering bias.";
        param_desc.additional_constraints = "The QCar 2 chassis sometimes has a bias in the steering so that driving the steering output with zero does not produce a zero angle on the wheels i.e., the car may not drive in a straight line when the steering output is set to zero. To adjust for this bias, the steer_bias option may be used to add a small offset to the steering output to eliminate this bias. The value specified should be in radians and may be positive or negative as appropriate. Suitable values are typically between 0.03 and 0.09.";
        this->declare_parameter("steer_bias", steer_bias, param_desc);

        param_desc.description = "Device_Type.";
        param_desc.additional_constraints = "This parameter allows you to switch between the ID of a physical and virtual QCar2";
        this->declare_parameter("device_type", device_type, param_desc);

        param_desc.description = "LED Strip Color";
        param_desc.additional_constraints = "This parameter allows you to set the LEDs for the QCar2";
        this->declare_parameter("led_color_id", 0, param_desc);


        // Parameters initialization
        try
        {
            parameter_cb = this->add_on_set_parameters_callback(std::bind(&QCar2::set_parameters_callback, this, _1));
        }
        catch (const std::bad_alloc& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error setting up parameters callback. %s", e.what());
            return;
        }

        // Actually get the parameters
        std::string device_type = this->get_parameter("device_type").as_string();
        std::string uri_param;
        std::string LED_uri_param;

        if (device_type.compare("physical")==0){
            uri_param = "0";
            LED_uri_param ="spi://localhost:1?memsize=420,word=8,baud=3333333,lsb=off,frame=1";
        }
        else if (device_type.compare("virtual")==0){
            uri_param = "0@tcpip://localhost:18960";
            LED_uri_param ="tcpip://localhost:18969";
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Invalid device type, stop node and input either virtual/physical...");
            return;
        }

        RCLCPP_INFO(this->get_logger(),"Current HIL URI for device is: %s", uri_param.c_str() );
        RCLCPP_INFO(this->get_logger(),"Current LED URI for device is: %s", LED_uri_param.c_str() );

        // Open the LED strip device
        result =  aaaf5050_mc_k12_open(LED_uri_param.c_str(), LED_STRIP_SIZE, &led_strip);
        if (result < 0)
        {
            msg_get_error_messageA(NULL, result, error_message, sizeof(error_message));
            RCLCPP_ERROR(this->get_logger(), "LED result message is: %s", error_message);
            return;
        }

        // Open the HIL "card"
        result = hil_open("qcar2", uri_param.c_str(), &card);
        if (result < 0)
        {
            msg_get_error_messageA(NULL, result, error_message, sizeof(error_message));
            RCLCPP_ERROR(this->get_logger(), "hil_open error: %s", error_message);
            return;
        }

        // Actually get the parameters
        std::map<std::string, double> params;
        std::map<std::string, double>::iterator it;

        // Get the Gyro params
        if (this->get_parameters({"gyro"}, params))
        {
            // RCLCPP_INFO(this->get_logger(), "Gyro parameters:");
            for (it = params.begin(); it != params.end(); it++)
            {
                // RCLCPP_INFO(this->get_logger(), "%s: %lf", it->first.c_str(), it->second);

                if (it->first.compare("fs") == 0)
                {
                    //RCLCPP_INFO(this->get_logger(), "This param is \"fs\" so assign to the right var");
                    gyro_fs = it->second;
                }
                else if (it->first.compare("rate") == 0)
                {
                    //RCLCPP_INFO(this->get_logger(), "This param is \"rate\" so assign to the right var");
                    gyro_rate = it->second;
                }
                else if (it->first.compare("bw") == 0)
                {
                    //RCLCPP_INFO(this->get_logger(), "This param is \"bw\" so assign to the right var");
                    gyro_bw = it->second;
                }
                else if (it->first.compare("ord") == 0)
                {
                    //RCLCPP_INFO(this->get_logger(), "This param is \"ord\" so assign to the right var");
                    gyro_ord = it->second;
                }
                // Any other gyro.* parameters won't get in here anyways because ROS will not pass
                // non declared parameters in.
            }
        }

        // Get the Accel params
        if (this->get_parameters({"accel"}, params))
        {
            //RCLCPP_INFO(this->get_logger(), "Accel parameters:");
            for (it = params.begin(); it != params.end(); it++)
            {
                //RCLCPP_INFO(this->get_logger(), "%s: %lf", it->first.c_str(), it->second);

                if (it->first.compare("fs") == 0)
                {
                    //RCLCPP_INFO(this->get_logger(), "This param is \"fs\" so assign to the right var");
                    accel_fs = it->second;
                }
                else if (it->first.compare("rate") == 0)
                {
                    //RCLCPP_INFO(this->get_logger(), "This param is \"rate\" so assign to the right var");
                    accel_rate = it->second;
                }
                else if (it->first.compare("bw") == 0)
                {
                    //RCLCPP_INFO(this->get_logger(), "This param is \"bw\" so assign to the right var");
                    accel_bw = it->second;
                }
                else if (it->first.compare("ord") == 0)
                {
                    //RCLCPP_INFO(this->get_logger(), "This param is \"ord\" so assign to the right var");
                    accel_ord = it->second;
                }
                // Any other accel.* parameters won't get in here anyways because ROS will not pass
                // non declared parameters in.
            }
        }

        temp_bw = this->get_parameter("temp_bw").as_double();
        //RCLCPP_INFO(this->get_logger(), "Parameter temp_bw = %lf", temp_bw);

        steer_bias = this->get_parameter("steer_bias").as_double();
        //RCLCPP_INFO(this->get_logger(), "Parameter temp_bw = %lf", temp_bw);

        std::ostringstream bso_stream;
        bso_stream << "gyro_fs=" << gyro_fs << ";"
                   << "gyro_rate=" << gyro_rate << ";"
                   << "gyro_bw=" << gyro_bw << ";"
                   << "gyro_ord=" << gyro_ord << ";"
                   << "accel_fs=" << accel_fs << ";"
                   << "accel_rate=" << accel_rate << ";"
                   << "accel_bw=" << accel_bw << ";"
                   << "accel_ord=" << accel_ord << ";"
                   << "temp_bw=" << temp_bw << ";"
                   << "steer_bias=" << steer_bias << ";"
                   << "enc0_dir=0;enc1_dir=0;enc2_dir=0";

        //RCLCPP_INFO(this->get_logger(), "bso is \"%s\".", bso_stream.str().c_str());
        result = hil_set_card_specific_options(card, bso_stream.str().c_str(), bso_stream.str().length());
        if (result < 0)
        {
            msg_get_error_messageA(NULL, result, error_message, sizeof(error_message));
            RCLCPP_ERROR(this->get_logger(), "hil_set_card_specific_options error: %s", error_message);
            return;
        }

        result = hil_watchdog_clear(card);
        if (result < 0 && result != -QERR_HIL_WATCHDOG_CLEAR)
        {
            msg_get_error_messageA(NULL, result, error_message, sizeof(error_message));
            RCLCPP_ERROR(this->get_logger(), "hil_watchdog_clear error: %s", error_message);
            return;
        }

        // Create the publishers
        battery_state_publisher_ = this->create_publisher<sensor_msgs::msg::BatteryState>("qcar2_battery", 1, pub_options);
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("qcar2_imu", 1, pub_options);
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("qcar2_joint", 1, pub_options);
        // User buttons S2, S3, S4 as bits 0, 1, 2 (1 = pressed); sent on change and once a second.
        buttons_publisher_ = this->create_publisher<std_msgs::msg::UInt8>("qcar2_user_buttons", 10, pub_options);

        // Create the subscribers
        led_cmd_subscriber_ = this->create_subscription<qcar2_interfaces::msg::BooleanLeds>("qcar2_led_cmd", 1, std::bind(&QCar2::led_command_callback, this, _1), sub_options);
        motor_cmd_subscriber_ = this->create_subscription<qcar2_interfaces::msg::MotorCommands>("qcar2_motor_speed_cmd", 1, std::bind(&QCar2::motor_command_callback, this, _1), sub_options);

        //RCLCPP_INFO(this->get_logger(), "driver_comm_sample_time.count: %ld", driver_comm_sample_time.count());
        timer_ = this->create_wall_timer(driver_comm_sample_time, std::bind(&QCar2::timer_callback, this), timer_cb_group_);

        //timer for controlling speed
        timer_speed_control_ = this->create_wall_timer(15ms, std::bind(&QCar2::speed_controller, this));

        // Any RGB colour for the LED strip (r, g, b 0..1, a = brightness 0..1), e.g. from the
        // web page; latched (transient local) so the last colour comes back after a restart.
        // A new led_color_id parameter value overrides it again.
        led_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions led_sub_options;
        led_sub_options.callback_group = led_cb_group_;
        led_color_subscriber_ = this->create_subscription<std_msgs::msg::ColorRGBA>(
            "qcar2_led_color", rclcpp::QoS(1).reliable().transient_local(),
            std::bind(&QCar2::led_color_callback, this, _1), led_sub_options);

        // Animated effects ("rainbow 1.0 0.8 255 0 0" = name speed brightness r g b; "none"
        // stops), drawn here ~30 times a second. Latched like the colour, which shows again when
        // the effect stops. A new led_color_id value stops the effect.
        led_effect_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "qcar2_led_effect", rclcpp::QoS(1).reliable().transient_local(),
            std::bind(&QCar2::led_effect_callback, this, _1), led_sub_options);
        timer_led_effect_ = this->create_wall_timer(33ms, std::bind(&QCar2::led_effect_timer, this),
                                                    led_cb_group_);
        effect_start_ = std::chrono::steady_clock::now();

        // LED strip timer in its own group: LED_Set() writes for 0.2 s, which used to hold up
        // the speed controller (same default group) on every colour change.
        timer_led_callback_ = this->create_wall_timer(500ms, std::bind(&QCar2::led_timer, this),
                                                      led_cb_group_);

        node_running = true;

    }

    ~QCar2()
    {
        // sending 0 commands to motor
        t_uint32 *channels = new t_uint32[2];
        t_double *buffer = new t_double[2];
        t_uint32 num_channels = 2;
        channels[0] = 1000;
        channels[1] = 11000;
        buffer[0] = 0.0;
        buffer[1] = 0.0;
        hil_write_other(card, channels, num_channels, buffer);

        int result;
        result = hil_close(card);

        if (result < 0)
            RCLCPP_ERROR(this->get_logger(), "Closing the card with error: %d", result);

        // desired LED color
        t_led_color color[LED_STRIP_SIZE];

        for(int i = 0; i < LED_STRIP_SIZE; i++)
        {
            color[i] = { 0, 0, 0 };
        }

        // Turn off the LED strip when we shut the node down
        aaaf5050_mc_k12_write(led_strip, color, LED_STRIP_SIZE);

        result = aaaf5050_mc_k12_close(led_strip);
        if (result < 0)
            RCLCPP_ERROR(this->get_logger(), "Closing LED strip with error: %i", result);

        node_running = false;
        RCLCPP_INFO(this->get_logger(), "qcar2 exit");
    }

private:

    void led_color_callback(const std_msgs::msg::ColorRGBA &msg)
    {
        auto to8 = [](double v, double a) {
            v = std::isfinite(v) ? v : 0.0;
            a = std::isfinite(a) ? a : 1.0;
            return static_cast<t_uint8>(std::lround(std::clamp(v, 0.0, 1.0) * std::clamp(a, 0.0, 1.0) * 255.0));
        };
        std::lock_guard<std::mutex> lock(led_mutex_);
        custom_color_ = {to8(msg.r, msg.a), to8(msg.g, msg.a), to8(msg.b, msg.a)};
        custom_color_pending_ = true;   // shown when no effect runs (the web sends "none" with a colour)
    }

    void led_effect_callback(const std_msgs::msg::String &msg)
    {
        std::istringstream in(msg.data);
        std::string name = "none";
        double speed = 1.0, bright = 1.0, r = 255, g = 255, b = 255;
        in >> name >> speed >> bright >> r >> g >> b;
        std::lock_guard<std::mutex> lock(led_mutex_);
        const bool was = effect_ != "none";
        effect_ = (name == "rainbow" || name == "cycle" || name == "chase" || name == "breathe") ? name : "none";
        effect_speed_ = std::clamp(std::isfinite(speed) ? speed : 1.0, 0.05, 10.0);
        effect_bright_ = std::clamp(std::isfinite(bright) ? bright : 1.0, 0.0, 1.0);
        effect_rgb_[0] = std::clamp(r, 0.0, 255.0);
        effect_rgb_[1] = std::clamp(g, 0.0, 255.0);
        effect_rgb_[2] = std::clamp(b, 0.0, 255.0);
        if (effect_ == "none" && was)
            custom_color_pending_ = true;   // back to the plain colour
        RCLCPP_INFO(this->get_logger(), "LED effect: %s (speed %.2f, brightness %.2f)",
                    effect_.c_str(), effect_speed_, effect_bright_);
    }

    static t_led_color hsv(double h, double v)
    {
        h = h - std::floor(h);
        const double x = h * 6.0;
        const int k = static_cast<int>(x) % 6;
        const double f = x - std::floor(x);
        double r = 0, g = 0, b = 0;
        switch (k) {
            case 0: r = 1; g = f; break;
            case 1: r = 1 - f; g = 1; break;
            case 2: g = 1; b = f; break;
            case 3: g = 1 - f; b = 1; break;
            case 4: r = f; b = 1; break;
            default: r = 1; b = 1 - f; break;
        }
        auto c = [v](double u) { return static_cast<t_uint8>(std::lround(std::clamp(u * v, 0.0, 1.0) * 255.0)); };
        return {c(r), c(g), c(b)};
    }

    void led_effect_timer()
    {
        std::string effect;
        double speed, bright, rgb[3];
        {
            std::lock_guard<std::mutex> lock(led_mutex_);
            effect = effect_;
            speed = effect_speed_;
            bright = effect_bright_;
            std::copy(effect_rgb_, effect_rgb_ + 3, rgb);
        }
        if (effect == "none")
            return;
        const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - effect_start_).count() * speed;
        const int n = LED_STRIP_SIZE;
        t_led_color color[LED_STRIP_SIZE];
        auto scaled = [&rgb](double v) {
            return t_led_color{static_cast<t_uint8>(std::lround(rgb[0] * v)), static_cast<t_uint8>(std::lround(rgb[1] * v)),
                               static_cast<t_uint8>(std::lround(rgb[2] * v))};
        };
        for (int i = 0; i < n; i++)
        {
            if (effect == "rainbow")        // the whole rainbow along the strip, running around
                color[i] = hsv(static_cast<double>(i) / n - 0.5 * t, bright);
            else if (effect == "cycle")     // the whole strip one colour, going through the rainbow
                color[i] = hsv(0.2 * t, bright);
            else if (effect == "chase")     // a light with a fading tail running around
            {
                const double head = std::fmod(0.5 * t, 1.0) * n;
                double d = std::fmod(head - i + n, static_cast<double>(n));   // distance behind the head
                color[i] = scaled(bright * std::max(0.0, 1.0 - d / 6.0));
            }
            else                            // breathe
                color[i] = scaled(bright * (0.08 + 0.92 * (0.5 - 0.5 * std::cos(2.0 * M_PI * 0.5 * t))));
        }
        t_error result = aaaf5050_mc_k12_write(led_strip, color, n);
        if (result < 0)
        {
            msg_get_error_messageA(NULL, result, error_message, sizeof(error_message));
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Error writing LED effect: %s", error_message);
        }
    }

    void led_timer()
    {
        this->get_parameter("led_color_id",led_color_id);
        if (response != led_color_id)
        {
            if (response != -1)
            {
                // the parameter was changed on purpose: it wins over a custom colour / effect
                std::lock_guard<std::mutex> lock(led_mutex_);
                custom_color_pending_ = false;
                effect_ = "none";
            }
            LED_Set();
            // RCLCPP_INFO(this->get_logger(),"Setting new LED Value %i",led_color_id);
            response = led_color_id;
        }
        t_led_color custom;
        bool pending;
        {
            std::lock_guard<std::mutex> lock(led_mutex_);
            pending = custom_color_pending_;
            custom = custom_color_;
            custom_color_pending_ = false;
        }
        bool effect_on;
        {
            std::lock_guard<std::mutex> lock(led_mutex_);
            effect_on = effect_ != "none";
        }
        if (pending && !effect_on)
        {
            LED_Write(custom);
        }
    }

    void LED_Write(t_led_color color_value)
    {
        t_led_color color[LED_STRIP_SIZE];
        for (int i = 0; i < LED_STRIP_SIZE; i++)
        {
            color[i] = color_value;
        }
        // Written repeatedly for 0.2 s, as LED_Set() does (single writes are sometimes missed).
        auto start_time = clock_.now();
        while ((clock_.now() - start_time).seconds() < 0.2)
        {
            t_error result = aaaf5050_mc_k12_write(led_strip, color, LED_STRIP_SIZE);
            if (result < 0)
            {
                msg_get_error_messageA(NULL, result, error_message, sizeof(error_message));
                RCLCPP_ERROR(this->get_logger(), "Error writing to LED strip error: %s", error_message);
                break;
            }
        }
    }

    void LED_Set()
    {
        //desired LED color
        t_led_color color[LED_STRIP_SIZE];
        t_led_color color_value  = {0,0,0};

        // color ID selection
        if (led_color_id == 0)
            {color_value = { 255, 0, 0 };}
        if (led_color_id == 1)
            {color_value = { 0, 255, 0 }; }
        if (led_color_id == 2)
            {color_value = { 0, 0, 255 };}
        if (led_color_id == 3)
            {color_value = { 255, 255, 0 };}
        if (led_color_id == 4)
            {color_value = { 0, 255, 255 };}
        if (led_color_id == 5)
            {color_value = { 255, 0, 255 };}

        // { 255, 0, 0 };        /* LED #0: red     */
        // { 0, 255, 0 },        /* LED #1: green   */
        // { 0, 0, 255 },        /* LED #2: blue    */
        // { 255, 255, 0 },      /* LED #3: yellow  */
        // { 0, 255, 255 },      /* LED #4: cyan    */
        // { 255, 0, 255 },      /* LED #5: magenta */

        for(int i = 0; i < LED_STRIP_SIZE; i++)
        {
            color[i] = color_value;
        }

        t_int result;
        auto start_time = clock_.now();
        rclcpp::Duration delta_time = clock_.now()-start_time;
        while ( delta_time.seconds() < 0.2)
        {
            delta_time = clock_.now()-start_time;
            result = aaaf5050_mc_k12_write(led_strip, color, LED_STRIP_SIZE);
            if (result < 0)
            {
                msg_get_error_messageA(NULL, result, error_message, sizeof(error_message));
                RCLCPP_ERROR(this->get_logger(), "Error writing to LED strip error: %s", error_message);
            }
        }
    }

    void speed_controller()
    {
        auto start_time = clock_.now();
        // time delta calculation
        rclcpp::Duration delta_time = start_time-end_time_;
        double measured_speed = 0;
        // method used for constructing a PD speed controller for QCar2
        //Convert desired linear speed to desired motor speed
        
        if (desired_speed != 0)
        {
            measured_speed = (joint_speed_measured/(720.0*4.0))*((13.0*19.0)/(70.0*30.0))*(2.0*M_PI)*0.033;
            speed_error = desired_speed-measured_speed;
            speed_error_integral += speed_error*delta_time.seconds();
            // motor_speed_cmd = motor_speed_cmd+ (speed_error*kp+((speed_error-prior_speed_error)/delta_time.seconds())*kd)*0.0047/battery_voltage;
            // prior_speed_error = speed_error;
            
            motor_speed_cmd = desired_speed*km/battery_voltage + kp*speed_error + ki*speed_error_integral;
            // prior_speed_error = (speed_error)*delta_time.seconds() + prior_speed_error;

            // clip pwm command to not exceed 0.3
            if (motor_speed_cmd>0.3)
                motor_speed_cmd =0.3;
            if (motor_speed_cmd<-0.3)
                motor_speed_cmd = -0.3;
            // check for motor deadband at PWM ~|0.03|
            if (motor_speed_cmd<0.01 && motor_speed_cmd >=0 && desired_speed > 0)
                motor_speed_cmd =0.01+motor_speed_cmd;
            if (motor_speed_cmd<0.0 && motor_speed_cmd >=-0.01&& desired_speed < 0)
                motor_speed_cmd =-0.01+motor_speed_cmd;

        }
        else
        {
            motor_speed_cmd = 0;
        }

        // motor channel mapping
        std::map<std::string, int> motor_channel_map {{"steering_angle", 1000},
                                                    {"motor_throttle", 11000}};

        size_t motor_commands_size = motor_channel_map.size();

        //values for writting command to HIL device
        t_uint32 *channels = new t_uint32[motor_commands_size];
        t_double *buffer = new t_double[motor_commands_size];
        t_uint32 num_channels = 2;

        channels[0] = 1000;
        channels[1] = 11000;

        buffer[0] = desired_steering;
        buffer[1] = motor_speed_cmd;

        t_error result;

        result = hil_write_other(card, channels, num_channels, buffer);
        if (result < 0)
        {
            msg_get_error_messageA(NULL, result, error_message, sizeof(error_message));
            RCLCPP_ERROR(this->get_logger(), "hil_write_other error: %s", error_message);
        }

        delete[] channels;
        delete[] buffer;

        // RCLCPP_INFO(this->get_logger(),"Measured Linear Speed is %f", measured_speed);
        // RCLCPP_INFO(this->get_logger(),"Speed Error is %f", speed_error);
        // RCLCPP_INFO(this->get_logger(),"Command is %f", motor_speed_cmd);
        // RCLCPP_INFO(this->get_logger(),"Time is %f", delta_time.seconds());

        end_time_ = start_time;
    }

    void timer_callback()
    {
        t_error result;
        rclcpp::Time hil_read_time;

        t_uint32 AIChannels[] = { 0,   // analog input 0
                                  1,   // analog inptu 1
                                  2,   // battery voltage
                                  3,   // electronics current
                                  4};  // motor current

        t_double AIBuffer[ARRAY_LENGTH(AIChannels)];

        t_uint32 ENChannels[] = {0,     // motor encoder
                                 1,     // user encoder 0
                                 2};    // user encoder 1

        t_int32 ENBuffer[ARRAY_LENGTH(ENChannels)];

        t_uint32 DIChannels[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, // bi-directional digital input
                                  11,   // user button 0
                                  12,   // user button 1
                                  13,   // user button 2
                                  14};  // over current condition

        t_boolean DIBuffer[ARRAY_LENGTH(DIChannels)];

        t_uint32 OIChannels[] = {3000, 3001, 3002,  // angular velocity (from gyroscope)
                                 4000, 4001, 4002,  // lineaear acceleration (from accelerometer)
                                 10000,             // IMU temperature
                                 14000};            // Motor Speed

        t_double OIBuffer[ARRAY_LENGTH(OIChannels)];

        // Read from QCar2 via HIL APIs
        result = hil_read(card,
                        AIChannels, ARRAY_LENGTH(AIChannels),
                        ENChannels, ARRAY_LENGTH(ENChannels),
                        DIChannels, ARRAY_LENGTH(DIChannels),
                        OIChannels, ARRAY_LENGTH(OIChannels),
                        AIBuffer,
                        ENBuffer,
                        DIBuffer,
                        OIBuffer
                        );
        if (result < 0)
        {
            msg_get_error_messageA(NULL, result, error_message, sizeof(error_message));
            RCLCPP_ERROR(this->get_logger(), "hil_read error: %s", error_message);
            return;
        }

        hil_read_time = this->get_clock()->now();

        /* Battery State message */
        auto battery_state = sensor_msgs::msg::BatteryState();
        battery_state.voltage = AIBuffer[2];
        battery_voltage = AIBuffer[2];
        battery_state.temperature = std::numeric_limits<double>::quiet_NaN();
        battery_state.current = std::numeric_limits<double>::quiet_NaN();
        battery_state.charge = std::numeric_limits<double>::quiet_NaN();
        battery_state.capacity = std::numeric_limits<double>::quiet_NaN();
        battery_state.design_capacity = 7.0;    // ??
        battery_state.percentage = std::numeric_limits<double>::quiet_NaN();
        battery_state.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
        battery_state.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
        battery_state.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
        battery_state.present = true;

        battery_state.header.stamp = hil_read_time;
        battery_state.header.frame_id = "base_link";
        battery_state_publisher_->publish(battery_state);

        /* Imu message */
        auto imu = sensor_msgs::msg::Imu();
        imu.linear_acceleration.x = OIBuffer[3];
        imu.linear_acceleration.y = OIBuffer[4];
        imu.linear_acceleration.z = OIBuffer[5];
        imu.angular_velocity.x = OIBuffer[0];
        imu.angular_velocity.y = OIBuffer[1];
        imu.angular_velocity.z = OIBuffer[2];

        imu.header.stamp = hil_read_time;
        imu.header.frame_id = "imu";
        imu_publisher_->publish(imu);

        /* Joint State message */
        auto joint_state = sensor_msgs::msg::JointState();
        joint_state.position.clear();
        joint_state.position.push_back(ENBuffer[0]);
        joint_state.velocity.clear();
        joint_state.velocity.push_back(OIBuffer[7]);
        joint_speed_measured = OIBuffer[7];
        joint_state.effort.clear();
        joint_state.effort.push_back(AIBuffer[4]);

        joint_state.header.stamp = hil_read_time;
        joint_state.header.frame_id = "base_link";
        joint_state_publisher_->publish(joint_state);

        /* User buttons */
        // The button inputs read 1 when released.
        int buttons = (DIBuffer[11] ? 0 : 1) | (DIBuffer[12] ? 0 : 2) | (DIBuffer[13] ? 0 : 4);
        if (buttons != last_buttons || hil_read_time.seconds() - last_buttons_time >= 1.0)
        {
            auto buttons_msg = std_msgs::msg::UInt8();
            buttons_msg.data = static_cast<uint8_t>(buttons);
            buttons_publisher_->publish(buttons_msg);
            last_buttons = buttons;
            last_buttons_time = hil_read_time.seconds();
        }
    }

    void led_command_callback(const qcar2_interfaces::msg::BooleanLeds &led_commands)
    {
        size_t led_commands_size = led_commands.led_names.size();
        std::map<std::string, int> led_channel_map {{"left_outside_brake_light",    11},
                                                    {"left_inside_brake_light",     12},
                                                    {"right_inside_brake_light",    13},
                                                    {"right_outside_brake_light",   14},
                                                    {"left_reverse_light",          15},
                                                    {"right_reverse_light",         16},
                                                    {"left_rear_signal",            17},
                                                    {"right_rear_signal",           18},
                                                    {"left_outside_headlight",      19},
                                                    {"left_middle_headlight",       20},
                                                    {"left_inside_headlight",       21},
                                                    {"right_inside_headlight",      22},
                                                    {"right_middle_headlight",      23},
                                                    {"right_outside_headlight",     24},
                                                    {"left_front_signal",           25},
                                                    {"right_front_signal",          26}};
        std::map<std::string, int>::iterator it;

        if (led_commands_size > led_channel_map.size())
        {
            RCLCPP_WARN(this->get_logger(), "In %s - size of BooleanLeds message must be between 0 and %ld, but received size is: %ld...ignore", __FUNCTION__, led_channel_map.size(), led_commands_size);
            return;
        }

        t_uint32 *channels = new t_uint32[led_commands_size];
        t_boolean *buffer = new t_boolean[led_commands_size];
        t_uint32 num_channels = 0;
        t_boolean valid_name = true;

        for (unsigned int i = 0; i < led_commands_size; i++)
        {
            it = led_channel_map.find(led_commands.led_names[i]);
            if (it == led_channel_map.end())
            {
                RCLCPP_WARN(this->get_logger(), "In %s - BooleanLeds message led_name %s is invalid...ignoring", __FUNCTION__, led_commands.led_names[i].c_str());
                valid_name = false;
                break;
            }

            channels[num_channels] = it->second;
            buffer[num_channels] = led_commands.values[i];
            num_channels++;
        }

        if (valid_name)
        {
            t_error result;
            result = hil_write_digital(card, channels, num_channels, buffer);
            if (result < 0)
            {
                msg_get_error_messageA(NULL, result, error_message, sizeof(error_message));
                RCLCPP_ERROR(this->get_logger(), "hil_write_digital error: %s", error_message);
            }
        }

        delete[] channels;
        delete[] buffer;
    }

    void motor_command_callback(const qcar2_interfaces::msg::MotorCommands &motor_commands)
    {
        size_t motor_commands_size = motor_commands.motor_names.size();
        std::map<std::string, int> motor_channel_map {{"steering_angle", 1000},
                                                      {"motor_throttle", 11000}};
        std::map<std::string, int>::iterator it;

        if (motor_commands_size > motor_channel_map.size())
        {
            RCLCPP_WARN(this->get_logger(), "In %s - size of MotorCommands message must be between 0 and %ld, but received size is: %ld...ignore", __FUNCTION__, motor_channel_map.size(), motor_commands_size);
            return;
        }

        for (unsigned int i = 0; i < motor_commands_size; i++)
        {
            it = motor_channel_map.find(motor_commands.motor_names[i]);
            if (it == motor_channel_map.end())
            {
                RCLCPP_WARN(this->get_logger(), "In %s - MotorCommands message command_name %s is invalid...ignoring", __FUNCTION__, motor_commands.motor_names[i].c_str());
                break;
            }

            if (i == 0)
                desired_steering = motor_commands.values[0];

            if (i == 1)
                desired_speed = motor_commands.values[1];
        }
    }

    rcl_interfaces::msg::SetParametersResult set_parameters_callback(const std::vector<rclcpp::Parameter> & parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;

        result.successful = true;

        // Loop through the parameters....can happen if set_parameters_atomically() is called
        for (const auto & parameter : parameters)
        {
            if (parameter.get_name().compare(0, 5, "gyro.") == 0)
            {
                if (node_running)
                {
                    result.successful = false;
                    result.reason = "Cannot change this parameter while node is running.";
                }
            }
            else if (parameter.get_name().compare(0, 6, "accel.") == 0)
            {
                if (node_running)
                {
                    result.successful = false;
                    result.reason = "Cannot change this parameter while node is running.";
                }
            }
            else if (parameter.get_name().compare("temp_bw") == 0)
            {
                if (node_running)
                {
                    result.successful = false;
                    result.reason = "Cannot change this parameter while node is running.";
                }
                else
                {
                    if ((parameter.as_double() < min_temp_bw) || (parameter.as_double() > max_temp_bw))
                    {
                        std::ostringstream error_stream;

                        error_stream << "temp_bw must be between " << min_temp_bw << " and " << max_temp_bw << ".";

                        result.successful = false;
                        result.reason = error_stream.str();
                    }
                }
            }
            else if (parameter.get_name().compare("steer_bias") == 0)
            {
                if (node_running)
                {
                    result.successful = false;
                    result.reason = "Cannot change this parameter while node is running.";
                }
            }
            else if (parameter.get_name().compare("device_type") == 0)
            {
                if (node_running)
                {
                    result.successful = false;
                    result.reason = "This parameter cannot change while node is running.";
                }
            }
            else if (parameter.get_name().compare("led_color_id") == 0)
            {
                if(node_running == false)
                {
                this->get_parameter("led_color_id",led_color_id);
                RCLCPP_INFO(this->get_logger(),"New LED ID is %i", led_color_id);
                LED_Set();
                }
            }
            else
            {
                result.successful = false;
                result.reason = "The parameter is invalid.";
            }
        }

        return result;
    }

    rclcpp::CallbackGroup::SharedPtr other_cb_group_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_led_callback_;
    rclcpp::CallbackGroup::SharedPtr led_cb_group_;
    rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr led_color_subscriber_;
    std::mutex led_mutex_;
    t_led_color custom_color_ = {0, 0, 0};
    bool custom_color_pending_ = false;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr led_effect_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_led_effect_;
    std::string effect_ = "none";
    double effect_speed_ = 1.0, effect_bright_ = 1.0, effect_rgb_[3] = {255, 255, 255};
    std::chrono::steady_clock::time_point effect_start_;

    //LED ID selector
    int led_color_id;
    int response = -1;


    // speed controller parameters
    double desired_rotation_speed = 0;
    double speed_error = 0;
    double speed_error_integral = 0;
    double kp = 0.1;
    double kd = 0;
    double ki = 1;
    double km = 0.0047; // v/rad/s
    double joint_speed_measured = 0.0;
    double battery_voltage = 0;
    double desired_speed= 0;
    double desired_steering = 0;
    double prior_speed_error =0;
    double motor_speed_cmd = 0;
    rclcpp::Clock clock_;
    rclcpp::Time end_time_;
    rclcpp::TimerBase::SharedPtr timer_speed_control_;

    t_aaaf5050_mc_k12 led_strip;

    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr buttons_publisher_;
    int last_buttons = -1;
    double last_buttons_time = 0.0;

    rclcpp::Subscription<qcar2_interfaces::msg::BooleanLeds>::SharedPtr led_cmd_subscriber_;
    rclcpp::Subscription<qcar2_interfaces::msg::MotorCommands>::SharedPtr motor_cmd_subscriber_;

    char error_message[512];
    t_card card;

    // parameters change callback
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr parameter_cb;

    // Sample time to get data from QCar2
    std::chrono::milliseconds driver_comm_sample_time{1};

    bool node_running = false;

    const t_double min_temp_bw = 5.0;
    const t_double max_temp_bw = 4000.0;

    // parameters
    t_double gyro_fs   = 250.0;
    t_double gyro_rate = 500.0;
    t_double gyro_bw   = 125.0;
    t_double gyro_ord  = 3.0;

    t_double accel_fs   = 16.0;
    t_double accel_rate = 1000.0;
    t_double accel_bw   = 250.0;
    t_double accel_ord  = 3.0;

    t_double temp_bw = 4000;
    t_double steer_bias = -0.035;
    std::string device_type = "physical";
};

int main(int argc, char * argv[])
{
    // Initialize the ROS environment
    rclcpp::init(argc, argv);

    // Instantiate the node
    rclcpp::Node::SharedPtr qcars_node = std::make_shared<QCar2>();

    // Get a multi-threaded executor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(qcars_node);

    RCLCPP_INFO(qcars_node->get_logger(), "Starting qcar2 loop...");
    executor.spin();
    RCLCPP_INFO(qcars_node->get_logger(), "qcar2 loop ended.\n");

    // Shutdown and exit
    rclcpp::shutdown();
    return 0;
}
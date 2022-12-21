class Drive
{
private:
    float max_speed;
    float speed;
    int angle;
    int stop_counter;
    float kp_, ki_, kd_;
    float p_error, i_error, d_error;

public:
    pair<int,int> calc_order(float theta, float l_pos, float r_pos, float roi_x_value, float roi_y_value);
    float pid_control(float err);
    Drive(float init_speed, float max_spd, float kp, float ki, float kd);
    ~Drive();
};


pair<int,int> Drive::calc_order(float theta, float l_pos, float r_pos, float roi_x_value, float roi_y_value)
{
    float x_center = roi_x_value / 2;
    float y_center = roi_y_value / 2;
    float lane_center = (l_pos + r_pos) / 2;

    // first = speed, second = angle
    pair<int,int> order(speed, 0);

    int theta_;
    float center_diff = lane_center - x_center;

    theta_ = cvRound((90 - theta) * 2.5f + center_diff);

    if (!theta_) return order;

    // steering
    order.second = abs(theta_) < 50 ? theta_ : (theta_ < 0) ? -50 : 50;
    

    // accel - break
    float break_press = 0;
    if (abs(order.second) > 15) {
        break_press = (order.second - 15) / 35;
    } else {
        break_press = -1;
    }

    speed = (speed - break_press < 1) ? 1 : (speed - break_press > max_speed) ? max_speed : speed - break_press;

    order.first = cvRound(speed);
    return stop_counter >= 10 ? pair<int,int> (0, 0) : order;
}

float Drive::pid_control(float err)
{
    d_error = err - p_error;
    p_error = err;
    i_error += err;

    return kp_ * p_error + ki_ * i_error + kd_ * d_error;
}

Drive::Drive(float init_speed, float max_spd, float kp, float ki, float kd)
{
    max_speed = max_spd;
    speed = init_speed;
    angle = 0;
    stop_counter = 10;

    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;

    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

Drive::~Drive()
{
}
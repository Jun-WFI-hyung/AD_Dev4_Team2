pair<int,int> Drive::calc_order(float theta, float l_pos, float r_pos, float roi_x_value, float roi_y_value)
{
    float x_center = roi_x_value / 2;
    float y_center = roi_y_value / 2;
    float lane_center = (l_pos + r_pos) / 2;

    // first = speed, second = angle
    pair<int,int> order(speed, 0);
    int theta_;

    if (theta < 0) {
        theta_ = angle;
    } else {        
        float center_diff = 50 * (lane_center - x_center) / 320;
        theta_ = cvRound((90 - theta) * 2.5f + center_diff);

        // steering
        angle = abs(theta_) < 50 ? theta_ : (theta_ < 0) ? -50 : 50;
        order.second = angle;
    }    

    // accel - break
    float break_press = 0;
    if (abs(order.second) > 15) {
        break_press = (order.second - 15) / 35;
    } else {
        break_press = -1;
    }

    speed = (speed - break_press < 1) ? 1 : (speed - break_press > max_speed) ? max_speed : speed - break_press;

    order.first = cvRound(speed);
    printf("\r speed : %d    //    angle : %f", speed, theta_);
    return stop_counter >= 10 ? pair<int,int> (0, 0) : order;
}
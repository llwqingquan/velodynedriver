// Copyright (C) 2019 Matthew Pitropov, Joshua Whitley
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef VELODYNE_DRIVER_TIME_CONVERSION_HPP
#define VELODYNE_DRIVER_TIME_CONVERSION_HPP

#include <ros/ros.h>
#include <ros/time.h>

/** @brief Function used to check that hour assigned to timestamp in conversion is
 * correct. Velodyne only returns time since the top of the hour, so if the computer clock
 * and the velodyne clock (gps-synchronized) are a little off, there is a chance the wrong
 * hour may be associated with the timestamp
 * 
 * @param stamp timestamp recovered from velodyne
 * @param nominal_stamp time coming from computer's clock
 * @return timestamp from velodyne, possibly shifted by 1 hour if the function arguments
 * disagree by more than a half-hour.
 */
ros::Time resolveHourAmbiguity(const ros::Time &stamp, const ros::Time &nominal_stamp) {
    const int HALFHOUR_TO_SEC = 1800;
    ros::Time retval = stamp;
    if (nominal_stamp.sec > stamp.sec) {
        if (nominal_stamp.sec - stamp.sec > HALFHOUR_TO_SEC) {
            retval.sec = retval.sec + 2*HALFHOUR_TO_SEC;
        }
    } else if (stamp.sec - nominal_stamp.sec > HALFHOUR_TO_SEC) {
        retval.sec = retval.sec - 2*HALFHOUR_TO_SEC;
    }
    return retval;
}

ros::Time rosTimeFromGpsTimestamp(const uint8_t * const data) {
    const int HOUR_TO_SEC = 3600;
    // time for each packet is a 4 byte uint
    // It is the number of microseconds from the top of the hour
    uint32_t usecs = (uint32_t) ( ((uint32_t) data[3]) << 24 |
                                  ((uint32_t) data[2] ) << 16 |
                                  ((uint32_t) data[1] ) << 8 |
                                  ((uint32_t) data[0] ));
    ros::Time time_nom = ros::Time::now(); // use this to recover the hour
    uint32_t cur_hour = time_nom.sec / HOUR_TO_SEC;
    ros::Time stamp = ros::Time((cur_hour * HOUR_TO_SEC) + (usecs / 1000000),
                                (usecs % 1000000) * 1000);
    stamp = resolveHourAmbiguity(stamp, time_nom);
    return stamp;
}

  /***********************************************************************
  * 函数说明:UTC转周秒 然后存储BUF
  * 输入参数:unsigned char *buf :  time :utc us
  * 输出参数:
  * 返回参数:周秒
  	打印调试代码:
	char *wday[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
	printf("%f,",tempD);
	printf("%d-%d-%d ", (p_tm->tm_year+1900), (p_tm->tm_mon+1), p_tm->tm_mday);
	printf("%s %d:%d:%d\n", wday[p_tm->tm_wday], p_tm->tm_hour, p_tm->tm_min, p_tm->tm_sec);
  ***********************************************************************/
 uint32_t timeConvert(unsigned long long time)
 {
	 struct tm *p_tm;
	 double tempD;
	 time_t timep;
	 
	 timep=time+18U;//转换
	 p_tm = gmtime(&timep); /*获取GMT时间*/
	 tempD=p_tm->tm_wday*86400+(p_tm->tm_hour)*3600+ p_tm->tm_min*60+p_tm->tm_sec;
	//  tempD+=(double)((double)(time%1000000)/1000000.0);
	//  tempD-=0.004;//IMU 获取的是上一次采样值
	 
	 return tempD;
 } 

ros::Time rosTimeFromUtcTimestamp(const uint8_t * const data, time_t time_utc_sec) {
    const int HOUR_TO_SEC = 3600;
    // time for each packet is a 4 byte uint
    // It is the number of microseconds from the top of the hour
    uint32_t usecs = (uint32_t) ( ((uint32_t) data[3]) << 24 |
                                  ((uint32_t) data[2] ) << 16 |
                                  ((uint32_t) data[1] ) << 8 |
                                  ((uint32_t) data[0] ));
    ros::Time time_nom;
    time_nom.sec = timeConvert(time_utc_sec); // conver to week sec
    time_nom.nsec = 0;
    uint32_t cur_hour = time_utc_sec / HOUR_TO_SEC;
    uint32_t sec = timeConvert((cur_hour * HOUR_TO_SEC) + (usecs / 1000000)); //convert to week sec
    // sec = time_utc_sec;
    ros::Time stamp = ros::Time(sec,(usecs % 1000000) * 1000);
    stamp = resolveHourAmbiguity(stamp, time_nom);
    return stamp;
}

#endif //VELODYNE_DRIVER_TIME_CONVERSION_HPP

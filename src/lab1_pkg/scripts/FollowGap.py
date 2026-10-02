#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from scipy.ndimage import uniform_filter1d
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class ReactiveFollowGap(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        self.angles = 0

        self.subscriber = self.create_subscription(LaserScan,lidarscan_topic, self.lidar_callback, 10)
        self.pub = self.create_publisher(AckermannDriveStamped, drive_topic, 5)
      

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
       
        window_size = 1
        proc_ranges = uniform_filter1d(ranges, size=window_size, mode='reflect')
        #for the blocked map proc_ranges[proc_ranges > 10] = 10
        proc_ranges[proc_ranges > 10] = 10
        return proc_ranges

    # def find_max_gap(self, free_space_ranges):
    #     """ Return the start index & end index of the max gap in free_space_ranges
    #     """
    #     max_start = -1
    #     max_end = -1
    #     max_gap_length = 0

    #     current_start = -1
    #     current_length = 0

    #     for i in range(len(free_space_ranges)):
    #             if free_space_ranges[i] > 2:
    #                 if current_start == -1:
    #                     current_start = i
    #                 current_length += 1

    #                 if current_length >= 3 and current_length > max_gap_length:
    #                     max_gap_length = current_length
    #                     max_start = current_start
    #                     max_end = i
    #             else:
    #                 current_start = -1
    #                 current_length = 0

    #     return (max_start, max_end)
    # for the blocked map threshold = 1
    # def find_max_gap(self, arr, threshold=1.2, min_length=2):
    #     max_gap_start = -1
    #     max_gap_end = -1
    #     current_start = -1
    #     max_length = 0

    #     for i, value in enumerate(arr):
    #         if value > threshold:
    #             if current_start == -1:
    #                 current_start = i
    #             current_length = i - current_start + 1
    #             if current_length >= min_length and current_length >= max_length:
    #                 max_gap_start = current_start
    #                 max_gap_end = i
    #                 max_length = current_length
    #         else:
    #             current_start = -1

    #     return max_gap_start, max_gap_end
    def find_max_gap(self, arr, threshold=1.2, min_length=3):
        max_gap_start = -1
        max_gap_end = -1
        current_start = -1
        max_sum = 0

        current_sum = 0
        current_length = 0

        for i, value in enumerate(arr):
            if value > threshold:
                if current_start == -1:
                    current_start = i
                    current_sum = value
                    current_length = 1
                else:
                    current_sum += value
                    current_length += 1

                if current_length >= min_length and current_sum > max_sum:
                    max_gap_start = current_start
                    max_gap_end = i
                    max_sum = current_sum
            else:
                current_start = -1
                current_sum = 0
                current_length = 0

        return max_gap_start, max_gap_end

    # def find_max_gap(self, arr, threshold=1.2, min_length=2, max_length=100):
    #     max_gap_start = -1
    #     max_gap_end = -1
    #     current_start = -1
    #     longest_length = 0

    #     for i, value in enumerate(arr):
    #         if value > threshold:
    #             if current_start == -1:
    #                 current_start = i
    #             current_length = i - current_start + 1
    #             if current_length >= min_length and current_length <= max_length and current_length > longest_length:
    #                 max_gap_start = current_start
    #                 max_gap_end = i
    #                 longest_length = current_length
    #             elif current_length > max_length:
    #                 current_start = -1  # Reset current_start if the current_length exceeds max_length
    #         else:
    #             current_start = -1

    #     return max_gap_start, max_gap_end


    
        
    # def find_best_point(self, start_i, end_i, ranges):
    #     """Start_i & end_i are start and end indicies of max-gap range, respectively
    #     Return index of best point in ranges
	#     Naive: Choose the furthest point within ranges and go there
    #     """

        
        
    #     free_space_ranges = ranges
    #     max_index = start_i
    #     for i in range(start_i + 1, end_i + 1):
    #         if free_space_ranges[i] >= free_space_ranges[max_index]:
    #             max_index = i

        

    #     return self.angles[max_index]
    def find_best_point(self, start_i, end_i, ranges):
        """
        Start_i & end_i are start and end indices of max-gap range, respectively.
        Return the index of the best point in ranges.
        Adjusted: Choose the midpoint within ranges and go there.
        """
        # Calculate the midpoint index
        midpoint_index = (start_i + end_i) // 2

        # Return the angle corresponding to the midpoint index
        return self.angles[midpoint_index]


    # def find_best_point(self, start_i, end_i, ranges):
    #     """Start_i & end_i are start and end indicies of max-gap range, respectively
    #     Return index of best point in ranges
	# Naive: Choose the furthest point within ranges and go there
    #     """

    #     return self.angles[np.argmax(ranges[start_i:end_i])+start_i]
    
    def extend_values(self, arr):
        i = 0

        while i < len(arr) - 1:
            if arr[i] - arr[i + 1] < -2.0:
                min_value = arr[i]
                # Ensure not to exceed array boundaries
                for j in range(1, 3):
                    if i + j < len(arr):
                        arr[i + j] = min_value
                i += 3
            elif arr[i] - arr[i + 1] > 2.0:
                min_value = arr[i + 1]
                # Ensure not to exceed array boundaries
                for j in range(0, 2):
                    if i - j >= 0:
                        arr[i - j] = min_value
                i += 1
            else:
                i += 1

        return arr

    # def extend_values(self,arr):
    #     i=0

    #     while i<len(arr)-1 :
    #         if arr[i]-arr[i+1] < -1.5  :
                
    #                 arr[i+1]=arr[i]
    #                 arr[i+2]=arr[i]
    #                 arr[i+3]=arr[i]
    #                 arr[i+4]=arr[i]
    #                 arr[i+5]=arr[i]
    #                 arr[i+6]=arr[i]
    #                 i=i+7
    #         elif arr[i]-arr[i+1] > 1.5 :
    #             arr[i]=arr[i+1]
    #             arr[i-1]=arr[i+1]
    #             arr[i-2]=arr[i+1]
    #             arr[i-3]=arr[i+1]
    #             arr[i-4]=arr[i+1]
    #             arr[i-5]=arr[i+1]
    #             arr[i-6]=arr[i+1]
                
    #             i+=1
    #         else :
    #             i+=1
    #     return arr
    #for the blocked map rb = 0.8

    def set_bubble(self, ranges, closest_point_idx, rb = 0.15):
        """Rb is bubble radius"""
        angle = self.angles[closest_point_idx]
        dtheta = np.arctan2(rb, ranges[closest_point_idx])

        bubble_idx = np.where(np.logical_and(self.angles > angle-dtheta, self.angles < angle+dtheta))

        ranges[bubble_idx] = 0

        return ranges




    
    # def set_smallest_points_to_zero(self,arr):
    #         if len(arr) == 0:
    #             return arr

    #         smallest_value = min(arr)
    #         indices_to_zero = set()

    #         # Find all indices of the smallest value
    #         for i, value in enumerate(arr):
    #             if value == smallest_value:
    #                 indices_to_zero.update(range(max(0, i-5), min(len(arr), i+6)))

    #         # Set the identified indices to zero
    #         for i in indices_to_zero:
    #             arr[i] = 0

    #         return arr

       

    def lidar_callback(self, data : LaserScan):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        cmd = AckermannDriveStamped()
        speed = 1.9
        min_angle = data.angle_min
        max_angle = data.angle_max
        self.angles = np.linspace(min_angle, max_angle, 1080)
        self.angles = self.angles[50:1030]
        # steering_angle1 = 0.1
        # steering_angle2 = 0.5
        # steering_angle3 = 1.0
        proc_ranges = self.preprocess_lidar(data.ranges[50:1030])
       
        
        # TODO:
        #Find closest point to LiDAR
        closest_point_idx = np.argmin(proc_ranges[np.nonzero(proc_ranges)])
        #Eliminate all points inside 'bubble' (set them to zero) 
        safety_ranges =self.extend_values(proc_ranges)
        

        #Find max length gap 
        
        sefety_ubdated_ranges = self.set_bubble(safety_ranges,closest_point_idx)
        start_i, end_i=self.find_max_gap(sefety_ubdated_ranges)


        #Find the best point in the gap 
        steering_angle_desired = self.find_best_point(start_i, end_i , sefety_ubdated_ranges)

        print(steering_angle_desired)
        #Publish Drive message
        # if 720>=max_index > 540 :
        #     cmd.drive.steering_angle = steering_angle1
        #     cmd.drive.speed = speed
        #     self.pub.publish(cmd)
          
        # elif 900 >= max_index > 720:
        #     cmd.drive.steering_angle = steering_angle2
        #     cmd.drive.speed = 0.5
        #     self.pub.publish(cmd)

        # elif 1080 >= max_index > 900:
        #     cmd.drive.steering_angle = steering_angle3
        #     cmd.drive.speed = 0.3
        #     self.pub.publish(cmd)

        # elif 360 <= max_index < 539:
        #     cmd.drive.steering_angle = -steering_angle1
        #     cmd.drive.speed = speed
        #     self.pub.publish(cmd)
            
        # elif 180 <= max_index < 360:
        #     cmd.drive.steering_angle = -steering_angle2
        #     cmd.drive.speed = 0.5
        #     self.pub.pu0.4ublish(cmd)

        # else :
        #     cmd.drive.steering_angle = 0.0
        #     cmd.drive.speed = speed
        #     self.pub.publish(cmd)

        cmd.drive.steering_angle = steering_angle_desired
        cmd.drive.speed = speed
        self.pub.publish(cmd)

        



def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
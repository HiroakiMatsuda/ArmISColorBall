#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

"""
 \file ArmISColorBall.py
 \brief ModuleDescription
 \date $Date$


"""
import sys
import time
sys.path.append(".")

# Import RTM module
import RTC
import OpenRTM_aist

import cv2.cv as cv
import ConfigParser as Conf
import time

# Import Service implementation class
# <rtc-template block="service_impl">

# </rtc-template>

# Import Service stub modules
# <rtc-template block="consumer_import">
# </rtc-template>


# This module's spesification
# <rtc-template block="module_spec">
armiscolorball_spec = ["implementation_id", "ArmISColorBall", 
		 "type_name",         "ArmISColorBall", 
		 "description",       "ModuleDescription", 
		 "version",           "1.0.0", 
		 "vendor",            "Matsuda Hiroaki", 
		 "category",          "ARM", 
		 "activity_type",     "STATIC", 
		 "max_instance",      "1", 
		 "language",          "Python", 
		 "lang_type",         "SCRIPT",
		 ""]
# </rtc-template>

class ArmISColorBall(OpenRTM_aist.DataFlowComponentBase):
	
	"""
	\class ArmISColorBall
	\brief ModuleDescription
	
	"""
	def __init__(self, manager):
		"""
		\brief constructor
		\param manager Maneger Object
		"""
		OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)

		self._d_pos = RTC.TimedLongSeq(RTC.Time(0,0),[])
		"""
		"""
		self._posOut = OpenRTM_aist.OutPort("pos", self._d_pos)


		


		# initialize of configuration-data.
		# <rtc-template block="init_conf_param">
		
		# </rtc-template>


		 
	def onInitialize(self):
		"""
		
		The initialize action (on CREATED->ALIVE transition)
		formaer rtc_init_entry() 
		
		\return RTC::ReturnCode_t
		
		"""
		# Bind variables and configuration variable
		
		# Set InPort buffers
		
		# Set OutPort buffers
		self.addOutPort("pos",self._posOut)
		
		# Set service provider to Ports
		
		# Set service consumers to Ports
		
		# Set CORBA Service Ports

		self.capture = cv.CaptureFromCAM(0)
                
                print('onInitialize')
                
		return RTC.RTC_OK

	
	def onActivated(self, ec_id):
		"""
	
		The activated action (Active state entry action)
		former rtc_active_entry()
	
		\param ec_id target ExecutionContext Id
	
		\return RTC::ReturnCode_t
	
		"""
		# Read ini file
		self.conf = Conf.SafeConfigParser()
                self.conf.read('ini/camera.ini')

                self.scale = int(self.conf.get('CALIB', 'scale'))
                self.robot = int(self.conf.get('CALIB', 'robot'))
                self.calibration_flag = self.conf.get('CALIB', 'flag')
                self.ball_size = int(self.conf.get('BALL', 'size'))
                self.grab = int(self.conf.get('BALL', 'grab'))
                
                red_h_low = int(self.conf.get('RED', 'h_low'))
                red_h_up = int(self.conf.get('RED', 'h_up'))
                red_s_low = int(self.conf.get('RED', 's_low'))
                red_s_up = int(self.conf.get('RED', 's_up'))
                red_v_low = int(self.conf.get('RED', 'v_low'))
                red_v_up = int(self.conf.get('RED', 'v_up'))

                green_h_low = int(self.conf.get('GREEN', 'h_low'))
                green_h_up = int(self.conf.get('GREEN', 'h_up'))
                green_s_low = int(self.conf.get('GREEN', 's_low'))
                green_s_up = int(self.conf.get('GREEN', 's_up'))
                green_v_low = int(self.conf.get('GREEN', 'v_low'))
                green_v_up = int(self.conf.get('GREEN', 'v_up'))

                blue_h_low = int(self.conf.get('BLUE', 'h_low'))
                blue_h_up = int(self.conf.get('BLUE', 'h_up'))
                blue_s_low = int(self.conf.get('BLUE', 's_low'))
                blue_s_up = int(self.conf.get('BLUE', 's_up'))
                blue_v_low = int(self.conf.get('BLUE', 'v_low'))
                blue_v_up = int(self.conf.get('BLUE', 'v_up'))

                pink_h_low = int(self.conf.get('PINK', 'h_low'))
                pink_h_up = int(self.conf.get('PINK', 'h_up'))
                pink_s_low = int(self.conf.get('PINK', 's_low'))
                pink_s_up = int(self.conf.get('PINK', 's_up'))
                pink_v_low = int(self.conf.get('PINK', 'v_low'))
                pink_v_up = int(self.conf.get('PINK', 'v_up'))

                try:
                        self.src = cv.QueryFrame(self.capture)

                except IOError:
                        print('Please check the connection of a USB Camera')

                size = cv.GetSize(self.src)


                self.center_x = size[0] / 2
                self.center_y = size[1] / 2
                self.img_center = (self.center_x, self.center_y)
                print self.img_center

                red  = [[red_h_low, red_h_up],
                        [red_s_low, red_s_up],
                        [red_v_low, red_v_up]]

                green  = [[green_h_low, green_h_up],
                        [green_s_low, green_s_up],
                        [green_v_low, green_v_up]]

                blue  = [[blue_h_low, blue_h_up],
                        [blue_s_low, blue_s_up],
                        [blue_v_low, blue_v_up]]

                pink  = [[pink_h_low, pink_h_up],
                        [pink_s_low, pink_s_up],
                        [pink_v_low, pink_v_up]]
                
                self.red_lut = self.calc_lut(red)
                self.green_lut = self.calc_lut(green)
                self.blue_lut = self.calc_lut(blue)
                self.pink_lut = self.calc_lut(pink)

                self.hsv = cv.CreateImage(cv.GetSize(self.src), 8, 3)
                self.ch_1 = cv.CreateImage(cv.GetSize(self.hsv), 8, 1)
                self.ch_2 = cv.CreateImage(cv.GetSize(self.hsv), 8, 1)
                self.ch_3 = cv.CreateImage(cv.GetSize(self.hsv), 8, 1)
                self.Mask = cv.CreateImage(cv.GetSize(self.hsv), 8, 1)
                self.dst = cv.CreateImage(cv.GetSize(self.src), 8, 3)
                self.gray = cv.CreateImage(cv.GetSize(self.src), 8, 1)

                cv.NamedWindow('ArmISColorBall', 1)

                print('Load INI file')

                data = [1000]
                self._d_pos.data = data
		OpenRTM_aist.setTimestamp(self._d_pos)
		self._posOut.write()
                print('ArmIS: Torque ON')

                time.sleep(0.5)

                data = [0]
                self._d_pos.data = data
		OpenRTM_aist.setTimestamp(self._d_pos)
		self._posOut.write()
                print('ArmIS: Initialize Position')

                print('onActivated')
	
		return RTC.RTC_OK
	
	def onDeactivated(self, ec_id):
		"""
	
		The deactivated action (Active state exit action)
		former rtc_active_exit()
	
		\param ec_id target ExecutionContext Id
	
		\return RTC::ReturnCode_t
	
		"""

		cv.DestroyAllWindows()

		data = [1001]
                self._d_pos.data = data
		OpenRTM_aist.setTimestamp(self._d_pos)
		self._posOut.write()
                print('ArmIS: Torque OFF')

		print('onDeactivated')

		return RTC.RTC_OK
	
	def onExecute(self, ec_id):
		"""
	
		The execution action that is invoked periodically
		former rtc_active_do()
	
		\param ec_id target ExecutionContext Id
	
		\return RTC::ReturnCode_t
	
		"""                        

		src = cv.QueryFrame(self.capture)
		cv.Smooth(src, src, cv.CV_GAUSSIAN, 5)
            
                gravity_red = self.color_extraction(src, self.red_lut)
                gravity_green = self.color_extraction(self.src, self.green_lut)
                gravity_blue = self.color_extraction(self.src, self.blue_lut)
                gravity_pink = self.color_extraction(self.src, self.pink_lut)

                if gravity_red != (0, 0):
                        cv.Circle(src, gravity_red, 10,
                                  cv.CV_RGB(255,0,153), 2)
                if gravity_green != (0, 0):
                        cv.Circle(self.src, gravity_green, 10,
                                  cv.CV_RGB(0,204,153), 2)
                if gravity_blue != (0, 0):
                        cv.Circle(self.src, gravity_blue, 10,
                                  cv.CV_RGB(51,0,255), 2)
                if gravity_pink != (0, 0):
                        cv.Circle(self.src, gravity_pink, 10,
                                  cv.CV_RGB(255,102,153), 2)

                if self.calibration_flag == 'ON':
                        cv.Line(src,
                                (self.center_x, self.center_y + self.scale),
                                (self.center_x, self.center_y - self.scale),
                                cv.CV_RGB(255,204,51), 1)
                        cv.Line(src,
                                (self.center_x - self.scale, self.center_y),
                                (self.center_x + self.scale, self.center_y),
                                cv.CV_RGB(255,204,51), 1)
                        cv.Circle(src,
                                  (self.center_x, self.center_y),
                                  self.scale, cv.CV_RGB(255,204,51), 1)
                        

                cv.ShowImage('ArmISColorBall', src)

                robo_red = self.transformation(gravity_red)
                robo_green = self.transformation(gravity_green)
                robo_blue = self.transformation(gravity_blue)
                robo_pink = self.transformation(gravity_pink)

                key = cv.WaitKey(30)

                # kye:'b'
                if key == 98:
                        data = [1, robo_blue[0], robo_blue[1], self.ball_size, self.grab]
                        self._d_pos.data = data
			OpenRTM_aist.setTimestamp(self._d_pos)
			self._posOut.write()
                        print('Send a ball position :Blue')
                        print robo_blue[0], robo_blue[1]

                # kye:'r'
                elif key == 114:
                        data = [1, robo_red[0], robo_red[1], self.ball_size, self.grab]
                        self._d_pos.data = data
			OpenRTM_aist.setTimestamp(self._d_pos)
			self._posOut.write()
                        print('Send a ball position :RED')
                        print robo_red[0], robo_red[1]

                # kye:'g'
                elif key == 103:
                        data = [1, robo_green[0], robo_green[1], self.ball_size, self.grab]
                        self._d_pos.data = data
			OpenRTM_aist.setTimestamp(self._d_pos)
			self._posOut.write()
                        print('Send a ball position :GREEN')
                        print robo_green[0], robo_green[1]

                # kye:'p'
                elif key == 112:
                        data = [1, robo_pink[0], robo_pink[1], self.ball_size, self.grab]
                        self._d_pos.data = data
			OpenRTM_aist.setTimestamp(self._d_pos)
			self._posOut.write()
                        print('Send a ball position :PINK')
                        print robo_pink[0], robo_pink[1]

                # key:'c'
                elif key == 99:
                        data = [2]
                        self._d_pos.data = data
			OpenRTM_aist.setTimestamp(self._d_pos)
			self._posOut.write()
                        print('Send a command :CATCH')

                # key:'i'
                elif key == 105:
                        data = [0]
                        self._d_pos.data = data
			OpenRTM_aist.setTimestamp(self._d_pos)
			self._posOut.write()
                        print('Send a command :INITIAL')

                # key:'o'
                elif key == 111:
                        data = [1000]
                        self._d_pos.data = data
			OpenRTM_aist.setTimestamp(self._d_pos)
			self._posOut.write()
                        print('Send a command :Torque ON')

                # key:'f'
                elif key == 102:
                        data = [1001]
                        self._d_pos.data = data
			OpenRTM_aist.setTimestamp(self._d_pos)
			self._posOut.write()
                        print('Send a command :Torque OFF')

                # key:'u'
                elif key == 117:
                        self.scale += 1
                        print('CLIB :%d' %self.scale)

                # key:'d'
                elif key == 100:
                        self.scale -= 1
                        print('CLIB :%d' %self.scale)

	
		return RTC.RTC_OK

	def calc_lut(self, hsv_range):
                lut = cv.CreateMat(256, 1, cv.CV_8UC3)
                val = [0, 0, 0]
                for i in range(256):
                    for j in range(3):
                        if hsv_range[j][0] <= hsv_range[j][1]:
                            if hsv_range[j][0] <= i and i <= hsv_range[j][1]:
                                val[j] = 255
                            else:
                                val[j] = 0
                        else:
                            if hsv_range[j][0] <= i or i <= hsv_range[j][1]:
                                val[j] = 255
                            else:
                                val[j] = 0
                
                    cv.Set1D(lut, i, cv.Scalar(val[0], val[1], val[2]))

                return lut
        
        def color_extraction(self, src, lut):        
                cv.Zero(self.hsv)
                cv.Zero(self.gray)
                cv.Zero(self.dst)
                cv.Zero(self.Mask)
                cv.Zero(self.ch_1)
                cv.Zero(self.ch_2)
                cv.Zero(self.ch_3)

                cv.CvtColor(src, self.hsv, cv.CV_BGR2HSV)
                cv.LUT(self.hsv, self.hsv, lut)
                cv.Split(self.hsv, self.ch_1, self.ch_2, self.ch_3, None)
                
                cv.And(self.ch_1, self.ch_2, self.Mask)
                cv.And(self.Mask, self.ch_3, self.Mask)
        
                cv.Copy(src, self.dst, self.Mask)
                cv.CvtColor(self.dst, self.gray, cv.CV_BGR2GRAY)

                mat = cv.GetMat(self.gray)
                mm = cv.Moments(mat, 1)
                m_00 = cv.GetSpatialMoment(mm, 0, 0)

                if m_00 != 0:
                        m_10 = cv.GetSpatialMoment(mm, 1, 0)
                        m_01 = cv.GetSpatialMoment(mm, 0, 1)
                        gravity_x = m_10 / m_00
                        gravity_y = m_01 / m_00
                else:
                        gravity_x = 0
                        gravity_y = 0

                return int(gravity_x), int(gravity_y)

        def transformation(self, img_gravity):
                mm_per_pixel = float(self.robot) / (self.scale * 2)
                x = int(round((img_gravity[0] - self.img_center[0]) * mm_per_pixel)) 
                y = -int(round((img_gravity[1] - self.img_center[1]) * mm_per_pixel))
                
                return (x, y)

#def ArmISColorBallInit(manager):
#    profile = OpenRTM_aist.Properties(defaults_str=armiscolorball_spec)
#    manager.registerFactory(profile,
#                            ArmISColorBall,
#                            OpenRTM_aist.Delete)
#
#def MyModuleInit(manager):
#    ArmISColorBallInit(manager)
#
#    # Create a component
#    comp = manager.createComponent("ArmISColorBall")

def main():
	mgr = OpenRTM_aist.Manager.init(sys.argv)
	mgr.activateManager()

	# Register component
	profile = OpenRTM_aist.Properties(defaults_str=armiscolorball_spec)
        mgr.registerFactory(profile,
                            ArmISColorBall,
                            OpenRTM_aist.Delete)
        
        comp = mgr.createComponent("ArmISColorBall")
	mgr.runManager()

if __name__ == "__main__":
	main()


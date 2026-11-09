"""Parameters of mapping (cartographer, RTAB-Map V-SLAM) and Nav2 that the web page can tune.

One place describes every tunable value: its range, unit, default and a short explanation
(in Thai, for demos and teaching). The same description drives

- the web page (sliders and the text next to them),
- the launch files, which read the saved values when mapping/navigation starts, and
- the live update of a running Nav2 (web_server.py sends the values to its nodes).

Only values that differ from the default are saved (~/.ros/qcar2_web_tuning.json), so a
better default in a later version reaches everybody who never touched that slider.

This module must stay importable without rclpy (the launch files import it).
"""

import copy
import json
import math
import os
import threading

import yaml

TUNING_PATH = '~/.ros/qcar2_web_tuning.json'
VSLAM_DB_PATH = '~/.ros/qcar2_rtabmap.db'        # database of the V-SLAM run in progress
VSLAM_LOC_DB_PATH = '~/.ros/qcar2_rtabmap_loc.db'  # working copy of a saved map for navigation
MAPS_DIR = '~/maps'

GROUPS = ('cartographer', 'vslam', 'nav')
GROUP_LABELS = {'cartographer': 'ทำแผนที่ด้วย Lidar (Cartographer)',
                'vslam': 'ทำแผนที่ด้วยกล้อง (V-SLAM, RTAB-Map)',
                'nav': 'นำทาง (Nav2)'}


def _p(key, label, default, lo=None, hi=None, step=None, unit='', kind='float', options=None,
       desc='', tip='', section='', live=False):
    return {'key': key, 'label': label, 'default': default, 'min': lo, 'max': hi, 'step': step,
            'unit': unit, 'kind': kind, 'options': options, 'desc': desc, 'tip': tip,
            'section': section, 'live': live}


# ---------------------------------------------------------------------------------------
# Cartographer (lidar SLAM). Applied when mapping starts: cartographer cannot change these
# while it runs.
# ---------------------------------------------------------------------------------------
_CG = 'ช่วงของ lidar'
_CGM = 'การจับคู่สแกน (scan matching)'
_CGL = 'ปิดลูป (loop closure)'
_CGG = 'แผนที่กริด'
CARTOGRAPHER = [
    _p('max_range', 'ระยะ lidar ที่ใช้', 3.5, 1.0, 12.0, 0.5, 'm', section=_CG,
       desc='ใช้เฉพาะจุดที่ใกล้กว่าระยะนี้สร้างแผนที่',
       tip='มาก = เห็นห้องกว้างขึ้นแต่จุดไกลมีสัญญาณรบกวน · น้อย = แผนที่คมกว่า แต่ต้องมีสิ่งของใกล้ ๆ ให้จับคู่'),
    _p('min_range', 'ระยะใกล้สุด', 0.12, 0.0, 1.0, 0.01, 'm', section=_CG,
       desc='ตัดจุดที่ใกล้กว่านี้ทิ้ง (เช่น ตัวรถเอง)',
       tip='มากเกินไปจะตัดผนังที่อยู่ชิดรถทิ้ง'),
    _p('use_imu', 'ใช้ไจโร (IMU)', True, kind='bool', section=_CG,
       desc='ใช้การหมุนจากไจโรช่วยเดาท่าทางระหว่างสแกน',
       tip='เปิดไว้ช่วยให้ตอนเลี้ยวไม่เพี้ยน'),
    _p('online_scan_matching', 'จับคู่สแกนละเอียด', True, kind='bool', section=_CGM,
       desc='ค้นหาตำแหน่งที่สแกนใหม่ตรงกับแผนที่มากที่สุดด้วยการลองหลายตำแหน่งรอบ ๆ',
       tip='เปิด = แม่นกว่าเมื่อ odometry ไม่ดี แต่กิน CPU · ปิด = เบา แต่เชื่อ odometry มากขึ้น'),
    _p('linear_window', 'หน้าต่างค้นหา (เส้นตรง)', 0.1, 0.02, 0.5, 0.01, 'm', section=_CGM,
       desc='ลองขยับสแกนไปรอบ ๆ ตำแหน่งเดิมไกลสุดเท่านี้',
       tip='มาก = แก้ความคลาดเคลื่อนใหญ่ได้ แต่ช้าและอาจจับคู่ผิด'),
    _p('angular_window', 'หน้าต่างค้นหา (มุม)', 20.0, 2.0, 60.0, 1.0, '°', section=_CGM,
       desc='ลองหมุนสแกนไปรอบ ๆ ทิศเดิมไกลสุดเท่านี้',
       tip='มาก = รับมือการหมุนเร็วได้ แต่ช้า'),
    _p('translation_weight', 'เชื่อการเลื่อนจาก odometry', 10.0, 0.0, 100.0, 1.0, section=_CGM,
       desc='น้ำหนักที่ถ่วงไม่ให้ตำแหน่งเลื่อนห่างจากที่เดาไว้',
       tip='มาก = เชื่อ odometry มาก (นิ่ง) · น้อย = เชื่อ lidar มาก (แก้ได้มากแต่สั่นง่าย)'),
    _p('rotation_weight', 'เชื่อการหมุนจากไจโร', 40.0, 0.0, 200.0, 1.0, section=_CGM,
       desc='น้ำหนักที่ถ่วงไม่ให้ทิศหมุนห่างจากที่เดาไว้',
       tip='มาก = เชื่อไจโรมาก · น้อย = เชื่อ lidar มาก'),
    _p('motion_angle', 'เก็บสแกนเมื่อหมุนเกิน', 0.1, 0.0, 5.0, 0.05, '°', section=_CGM,
       desc='สแกนจะถูกเก็บเข้าแผนที่เมื่อรถหมุนเกินค่านี้ (หรือขยับ/เวลาเกิน)',
       tip='น้อย = เก็บถี่ แผนที่ละเอียดแต่หนัก · มาก = เก็บห่าง เบากว่า'),
    _p('motion_distance', 'เก็บสแกนเมื่อขยับเกิน', 0.2, 0.0, 1.0, 0.01, 'm', section=_CGM,
       desc='เก็บสแกนเมื่อรถขยับเกินระยะนี้',
       tip='น้อย = ละเอียด · มาก = ประหยัด'),
    _p('submap_scans', 'จำนวนสแกนต่อ submap', 90, 20, 250, 5, '', kind='int', section=_CGM,
       desc='แผนที่ถูกแบ่งเป็นชิ้นย่อย (submap) ชิ้นละกี่สแกน',
       tip='มาก = ชิ้นใหญ่ ปิดลูปน้อยครั้งแต่แต่ละชิ้นเพี้ยนสะสมได้ · น้อย = ชิ้นเล็ก ปรับแก้ถี่'),
    _p('loop_closure', 'เปิดการปิดลูป', True, kind='bool', section=_CGL,
       desc='เมื่อรถกลับมาที่เดิม จะแก้ความคลาดเคลื่อนที่สะสมทั้งแผนที่',
       tip='ปิดเพื่อดูว่าแผนที่เพี้ยนสะสมอย่างไร (ใช้สอนได้ดี)'),
    _p('optimize_every', 'ปรับแต่งทั้งกราฟทุก', 90, 5, 300, 5, 'โหนด', kind='int', section=_CGL,
       desc='ทุกกี่ตำแหน่งที่เก็บ จะคำนวณแก้แผนที่ทั้งหมดใหม่',
       tip='น้อย = แผนที่ถูกแก้บ่อยแต่กิน CPU · มาก = แก้ช้า'),
    _p('loop_min_score', 'เกณฑ์ปิดลูป', 0.65, 0.3, 0.95, 0.01, section=_CGL,
       desc='คะแนนความเหมือนขั้นต่ำที่จะถือว่า "เคยมาที่นี่แล้ว"',
       tip='ต่ำ = ปิดลูปง่าย แต่เสี่ยงต่อผนังซ้ำ ๆ (แผนที่บิด) · สูง = เข้มงวด อาจไม่ปิดลูปเลย'),
    _p('loop_sampling', 'สัดส่วนที่ตรวจปิดลูป', 0.3, 0.05, 1.0, 0.05, section=_CGL,
       desc='ตรวจกี่ส่วนของโหนดว่าเคยเห็นที่เดิมไหม',
       tip='มาก = หาลูปได้ครบแต่กิน CPU'),
    _p('resolution', 'ขนาดช่องกริด', 0.05, 0.02, 0.15, 0.01, 'm', section=_CGG,
       desc='แต่ละช่องของแผนที่กว้างเท่านี้',
       tip='เล็ก = ละเอียดแต่ไฟล์/CPU มาก · ใหญ่ = หยาบ เบา'),
    _p('hit_probability', 'ความมั่นใจว่า "มีสิ่งกีดขวาง"', 0.55, 0.51, 0.9, 0.01, section=_CGG,
       desc='ทุกครั้งที่ lidar ชนผนัง ช่องนั้นมีโอกาสถูกครอบครองเพิ่มขึ้นเท่านี้',
       tip='มาก = ผนังขึ้นเร็วและเข้ม · น้อย = ต้องเห็นหลายรอบ (กรองสัญญาณรบกวนได้)'),
    _p('miss_probability', 'ความมั่นใจว่า "โล่ง"', 0.49, 0.1, 0.49, 0.01, section=_CGG,
       desc='ช่องที่ลำแสงผ่านไปมีโอกาสถูกครอบครองลดลงเท่านี้',
       tip='น้อย = พื้นที่โล่งถูกล้างเร็ว (ของที่เคลื่อนที่หายเร็ว) · มาก = ล้างช้า'),
]

# ---------------------------------------------------------------------------------------
# V-SLAM: RTAB-Map with the RealSense colour + depth images. Applied when mapping starts.
# ---------------------------------------------------------------------------------------
_VC = 'ภาพจากกล้อง'
_VF = 'จุดเด่นในภาพ (feature)'
_VL = 'การปิดลูปด้วยภาพ'
_VU = 'การอัปเดตแผนที่'
_VG = 'แผนที่กริด 2 มิติ'
_VA = 'ขั้นสูง (สอบเทียบกล้อง)'
VSLAM = [
    _p('image_width', 'ความกว้างภาพ', 424, kind='enum', unit='px', section=_VC,
       options=[[320, '320 (เบามาก)'], [424, '424 (แนะนำ)'], [640, '640 (ละเอียด)']],
       desc='ย่อภาพ RealSense (1280×720) ก่อนส่งให้ SLAM',
       tip='ใหญ่ = เห็นรายละเอียดมากขึ้น จับคู่แม่นขึ้น แต่ Jetson ทำงานหนักและแผนที่ช้าลง'),
    _p('rate', 'จำนวนภาพต่อวินาที', 8.0, 2.0, 15.0, 1.0, 'Hz', section=_VC,
       desc='ส่งภาพให้ SLAM วินาทีละกี่ภาพ',
       tip='มาก = ติดตามไม่ขาดตอนเมื่อรถเร็ว แต่กิน CPU · น้อย = เบา ต้องขับช้า'),
    _p('depth_range', 'ระยะ depth ที่ใช้', 4.0, 1.0, 6.0, 0.5, 'm', section=_VC,
       desc='ตัดจุด depth ที่ไกลกว่านี้ (D435 ยิ่งไกลยิ่งคลาดเคลื่อน)',
       tip='มาก = เห็นไกลแต่เสียงรบกวนมาก · น้อย = แผนที่สะอาดกว่า'),
    _p('max_features', 'จำนวน feature สูงสุดต่อภาพ', 600, 200, 1500, 50, '', kind='int', section=_VF,
       desc='จำนวนจุดเด่น (มุม/ลวดลาย) ที่ดึงจากภาพแต่ละภาพไว้จำสถานที่',
       tip='มาก = จำสถานที่ได้แม่นขึ้นและทนแสงเปลี่ยน แต่ช้า · น้อย = เร็ว แต่ผนังเรียบ ๆ จะหาที่เดิมไม่เจอ'),
    _p('feature_type', 'ชนิด feature', 8, kind='enum', section=_VF,
       options=[[8, 'GFTT + ORB (สมดุล)'], [2, 'ORB (เร็ว)'], [6, 'GFTT + BRIEF (เบามาก)'],
                [1, 'SIFT (แม่นสุด ช้าสุด)']],
       desc='วิธีหาและบรรยายจุดเด่นในภาพ',
       tip='SIFT ทนแสง/มุมมองดีที่สุด แต่หนัก · ORB/BRIEF เร็วเหมาะกับ Jetson'),
    _p('feature_max_depth', 'ใช้ feature ไกลสุด', 4.0, 1.0, 8.0, 0.5, 'm', section=_VF,
       desc='ข้าม feature ที่ depth ระบุว่าไกลกว่านี้',
       tip='ใกล้ = ตำแหน่ง 3 มิติของ feature แม่น · ไกล = ได้ feature มากขึ้นในห้องกว้าง'),
    _p('detection_rate', 'ตรวจสอบสถานที่ทุกกี่ครั้ง/วินาที', 1.0, 0.5, 5.0, 0.5, 'Hz', section=_VL,
       desc='ความถี่ที่จะเพิ่มโหนดใหม่และเทียบภาพกับที่เคยเห็น',
       tip='มาก = ปิดลูปเร็วแต่กิน CPU และฐานข้อมูลโตเร็ว'),
    _p('loop_threshold', 'เกณฑ์ปิดลูป', 0.11, 0.05, 0.6, 0.01, section=_VL,
       desc='ความมั่นใจขั้นต่ำว่า "ภาพนี้เหมือนที่เคยเห็น"',
       tip='ต่ำ = ปิดลูปง่าย (เสี่ยงปิดผิด แผนที่บิด) · สูง = เข้มงวด อาจไม่ปิดลูป'),
    _p('min_inliers', 'จำนวนจุดตรงกันขั้นต่ำ', 20, 8, 80, 1, '', kind='int', section=_VL,
       desc='ต้องมีจุดเด่นที่ตรงกันอย่างน้อยกี่จุด ถึงจะเชื่อว่าสองภาพคือที่เดียวกัน',
       tip='สูง = ปิดลูปผิดน้อย แต่ปิดได้ยาก · ต่ำ = ปิดง่ายแต่เสี่ยง'),
    _p('optimize_max_error', 'ยอมรับความคลาดเคลื่อนของลูป', 3.0, 0.0, 10.0, 0.5, '×', section=_VL,
       desc='ถ้าปิดลูปแล้วแผนที่ต้องบิดเกินกี่เท่าของความไม่แน่นอน จะปฏิเสธลูปนั้น (0 = ไม่ตรวจ)',
       tip='ต่ำ = กันลูปผิดได้ดี · สูงหรือ 0 = ยอมรับทุกลูป'),
    _p('linear_update', 'เพิ่มโหนดเมื่อขยับเกิน', 0.1, 0.0, 0.5, 0.01, 'm', section=_VU,
       desc='รถต้องขยับเกินระยะนี้ถึงจะเพิ่มภาพใหม่เข้าแผนที่ (0 = ทุกภาพ)',
       tip='น้อย = ละเอียดแต่ฐานข้อมูลโต · มาก = ประหยัด แต่ระหว่างทางอาจมีช่องว่าง'),
    _p('angular_update', 'เพิ่มโหนดเมื่อหมุนเกิน', 6.0, 0.0, 30.0, 0.5, '°', section=_VU,
       desc='รถต้องหมุนเกินมุมนี้ถึงจะเพิ่มภาพใหม่',
       tip='น้อย = ตามการเลี้ยวละเอียด · มาก = ประหยัด'),
    _p('grid_source', 'สร้างแผนที่กริดจาก', 'depth', kind='enum', section=_VG,
       options=[['depth', 'กล้อง depth (V-SLAM ล้วน)'], ['lidar', 'Lidar (กล้องช่วยปิดลูปอย่างเดียว)'],
                ['both', 'กล้อง + Lidar']],
       desc='แหล่งข้อมูลที่ใช้วาดผนัง/สิ่งกีดขวางในแผนที่ 2 มิติ',
       tip='depth = มุมมองแคบ (~87°) ต้องขับช้า ๆ หันรอบ ๆ · lidar = รอบทิศ แม่นกว่า · both = ครบที่สุด'),
    _p('cell_size', 'ขนาดช่องกริด', 0.05, 0.02, 0.15, 0.01, 'm', section=_VG,
       desc='แต่ละช่องของแผนที่กว้างเท่านี้',
       tip='เล็ก = ละเอียด แต่ depth รบกวนมองเห็นเป็นจุดเดี่ยว ๆ · ใหญ่ = เรียบ'),
    _p('ground_height', 'ความสูงที่ถือเป็นพื้น', 0.05, 0.01, 0.2, 0.01, 'm', section=_VG,
       desc='จุดจาก depth ที่สูงจากพื้นไม่เกินนี้ถือเป็นพื้น ไม่ใช่สิ่งกีดขวาง',
       tip='ต่ำเกินไป = พื้นขรุขระกลายเป็นสิ่งกีดขวาง · สูงเกินไป = ของเตี้ย ๆ หายไป'),
    _p('obstacle_height', 'ความสูงสิ่งกีดขวางสูงสุด', 0.6, 0.2, 2.0, 0.05, 'm', section=_VG,
       desc='จุดที่สูงกว่านี้ถือว่าอยู่เหนือรถ (เช่น เพดาน โต๊ะ) ไม่ใส่ในแผนที่',
       tip='ต่ำ = ไม่นับโต๊ะ/ชั้นวางที่ลอดได้ · สูง = นับทุกอย่างที่สูงจากพื้น'),
    _p('noise_radius', 'กรองจุดโดดเดี่ยว (รัศมี)', 0.1, 0.0, 0.3, 0.01, 'm', section=_VG,
       desc='ลบจุดสิ่งกีดขวางที่ไม่มีเพื่อนบ้านอยู่ใกล้ (0 = ไม่กรอง)',
       tip='มาก = แผนที่สะอาดแต่อาจลบของเล็ก ๆ · น้อย = เก็บทุกจุดรวมสัญญาณรบกวน'),
    _p('ray_tracing', 'ล้างพื้นที่โล่งตามลำแสง', True, kind='bool', section=_VG,
       desc='ช่องที่อยู่ระหว่างกล้องกับสิ่งกีดขวางจะถูกทำเครื่องหมายว่าโล่ง',
       tip='เปิด = ของที่ย้ายออกจะหายไปจากแผนที่ · ปิด = เบากว่า แต่รอยเก่าไม่หาย'),
    _p('color_hfov', 'มุมมองกล้องสี (แนวนอน)', 69.4, 40.0, 100.0, 0.5, '°', section=_VA,
       desc='ใช้คำนวณ intrinsic ของภาพสี (D435 ≈ 69°)',
       tip='ถ้าแผนที่ยืด/หดผิดสัดส่วน ลองปรับ'),
    _p('depth_hfov', 'มุมมองกล้อง depth (แนวนอน)', 87.0, 50.0, 110.0, 0.5, '°', section=_VA,
       desc='depth ของ D435 กว้างกว่าภาพสี จึงต้องจัดแนวให้ตรงกันก่อน (≈ 87°)',
       tip='ถ้าผนังใน depth กับภาพสีเหลื่อมกัน ลองปรับ'),
    _p('register_depth', 'จัด depth ให้ตรงภาพสี', True, kind='bool', section=_VA,
       desc='RealSense ส่งภาพสีกับ depth แยกกันโดยไม่จัดแนว จึงต้องปรับ depth ให้ตรงกับภาพสี',
       tip='ปิดเฉพาะเมื่อมั่นใจว่า depth ถูกจัดแนวมาแล้ว'),
]

# ---------------------------------------------------------------------------------------
# Nav2. Applied live to the running nodes where Nav2 allows it, and always at the next start.
# ---------------------------------------------------------------------------------------
_NS = 'ความเร็วและการเลี้ยว'
_NG = 'ความแม่นยำที่เป้าหมาย'
_NC = 'การหลบสิ่งกีดขวาง'
_NP = 'การวางเส้นทาง'
_NM = 'ตัวควบคุม MPPI (วิธีขับตามเส้นทาง)'
_NA = 'การหาตำแหน่งรถ (AMCL, ใช้กับแผนที่ Lidar)'
NAV = [
    _p('vx_max', 'ความเร็วเดินหน้าสูงสุด', 0.1, 0.05, 0.5, 0.01, 'm/s', section=_NS, live=True,
       desc='Nav2 จะไม่สั่งรถเร็วกว่านี้',
       tip='สาธิตใกล้คนใช้ 0.1–0.15 · เร็วเกินไปรถจะเลี้ยวไม่ทันและเฉี่ยวผนัง'),
    _p('vx_min', 'ความเร็วถอยหลังสูงสุด', -0.1, -0.4, 0.0, 0.01, 'm/s', section=_NS, live=True,
       desc='ความเร็วถอยที่ยอมให้ (ค่าลบ)',
       tip='0 = ห้ามถอยเลย รถจะไม่ถอยออกจากมุมแคบ'),
    _p('wz_max', 'อัตราเลี้ยวสูงสุด', 1.0, 0.3, 2.0, 0.05, 'rad/s', section=_NS, live=True,
       desc='การหมุนตัวสูงสุดที่ยอมให้',
       tip='รถจริงเลี้ยวได้จำกัดด้วยมุมล้อ ค่านี้มักไม่ใช่ตัวจำกัด'),
    _p('min_turning_r', 'รัศมีเลี้ยวต่ำสุดที่วางแผน', 0.3, 0.2, 1.5, 0.05, 'm', section=_NS, live=True,
       desc='ตอนวางเส้นทาง Nav2 คิดว่ารถเลี้ยวแคบสุดได้เท่านี้ (รถจริงที่ล้อเลี้ยว 0.5 rad ≈ 0.47 m)',
       tip='ถ้าน้อยกว่ารถจริง เส้นทางจะเลี้ยวแคบเกินกว่าที่รถทำได้ รถจะวิ่งออกนอกเส้น'),
    _p('xy_goal_tolerance', 'ถึงเป้าหมายเมื่อห่างไม่เกิน', 0.25, 0.05, 1.0, 0.05, 'm', section=_NG, live=True,
       desc='รัศมีรอบเป้าหมายที่ถือว่า "ถึงแล้ว"',
       tip='น้อย = จอดตรงจุดแต่รถวนหาจุดนานและอาจไม่ถึง · มาก = ถึงเร็วแต่คลาดจากจุด'),
    _p('yaw_goal_tolerance', 'คลาดทิศที่ยอมรับ', 0.25, 0.05, 3.14, 0.05, 'rad', section=_NG, live=True,
       desc='ทิศหน้ารถที่ต่างจากที่ลากไว้ได้ไม่เกินเท่านี้',
       tip='รถเลี้ยวกลับตัวยาก ถ้าตั้งน้อยมากรถอาจวนหาทิศนาน (3.14 = ไม่สนทิศ)'),
    _p('robot_radius', 'รัศมีรถ (สำหรับคำนวณชน)', 0.3, 0.1, 0.5, 0.01, 'm', section=_NC, live=True,
       desc='Nav2 มองรถเป็นวงกลมรัศมีเท่านี้ (รถจริงกว้างครึ่งหนึ่ง ≈ 0.1 m ยาว 0.43 m)',
       tip='มาก = ปลอดภัยแต่ไม่ยอมลอดช่องแคบ · น้อย = ลอดช่องแคบได้แต่เฉี่ยวง่าย'),
    _p('inflation_radius', 'ระยะเผื่อรอบสิ่งกีดขวาง', 0.1, 0.0, 1.0, 0.05, 'm', section=_NC, live=True,
       desc='ขยายสิ่งกีดขวางออกไปเท่านี้ เพื่อให้เส้นทางเลี่ยงห่างผนัง',
       tip='มาก = วิ่งกลางทางห่างผนัง แต่ทางแคบอาจกลายเป็น "ไปไม่ได้" · น้อย = ชิดผนัง'),
    _p('cost_scaling_local', 'ความชันของค่าเผื่อ (local)', 3.0, 0.5, 10.0, 0.5, '', section=_NC, live=True,
       desc='ยิ่งสูง ค่าเสี่ยงยิ่งลดเร็วเมื่อห่างจากผนัง (แผนที่ในระยะใกล้ตัวรถ)',
       tip='สูง = ผนังมีผลแค่ใกล้ ๆ · ต่ำ = ผนังมีผลไกลขึ้น'),
    _p('cost_scaling_global', 'ความชันของค่าเผื่อ (global)', 1.0, 0.5, 10.0, 0.5, '', section=_NC, live=True,
       desc='เหมือนข้างบน แต่ใช้กับแผนที่ทั้งหมดตอนวางเส้นทางระยะไกล',
       tip='สูง = เส้นทางชิดมุมได้ · ต่ำ = เส้นทางเลี่ยงผนังไกลขึ้น'),
    _p('obstacle_range', 'เห็นสิ่งกีดขวางไกลสุด', 2.5, 1.0, 6.0, 0.5, 'm', section=_NC,
       desc='ใช้จุด lidar ที่ใกล้กว่านี้ใส่ลงแผนที่สิ่งกีดขวาง (ต้องรีสตาร์ท Nav2)',
       tip='มาก = เห็นล่วงหน้า แต่ฝุ่น/สัญญาณรบกวนไกล ๆ ก็เข้ามา'),
    _p('planner_tolerance', 'ยอมให้เป้าคลาดจากเส้นทาง', 0.5, 0.0, 2.0, 0.1, 'm', section=_NP, live=True,
       desc='ถ้าเป้าหมายอยู่ในที่ที่ไปไม่ได้ จะหาจุดใกล้ ๆ ที่ไปได้ในรัศมีนี้',
       tip='มาก = ยอมไปใกล้ ๆ ได้ · น้อย = ต้องไปตรงจุดที่คลิก ไม่งั้นล้มเหลว'),
    _p('use_astar', 'ใช้ A* วางเส้นทาง', False, kind='bool', section=_NP, live=True,
       desc='เลือกอัลกอริทึมค้นหา: A* (ตรงไปหาเป้า) หรือ Dijkstra (ค้นรอบด้าน)',
       tip='A* เร็วกว่าในแผนที่ใหญ่ · เส้นทางที่ได้แทบเหมือนกัน'),
    _p('allow_unknown', 'ให้ผ่านพื้นที่ที่ยังไม่รู้จัก', True, kind='bool', section=_NP, live=True,
       desc='อนุญาตให้เส้นทางตัดผ่านช่องที่ยังไม่เคยสแกนในแผนที่',
       tip='เปิด = หาทางได้ง่ายขึ้น · ปิด = วิ่งเฉพาะที่แผนที่เคยเห็นว่าโล่งเท่านั้น'),
    _p('mppi_batch', 'จำนวนเส้นทางทดลอง (batch)', 2000, 500, 4000, 100, '', kind='int', section=_NM, live=True,
       desc='ทุกรอบ MPPI สุ่มลองเส้นทางสั้น ๆ กี่เส้น แล้วเลือกเส้นที่ดีที่สุด',
       tip='มาก = ขับนุ่มและหลบเก่งขึ้น แต่กิน CPU · น้อย = เบา แต่สั่นกระตุก'),
    _p('mppi_steps', 'มองไปข้างหน้ากี่ก้าว', 75, 30, 120, 5, '', kind='int', section=_NM, live=True,
       desc='แต่ละเส้นทางทดลองมองไปข้างหน้ากี่ก้าว (1 ก้าว = 0.05 วินาที)',
       tip='มาก = มองไกลขึ้น วางแผนเลี้ยวล่วงหน้า · น้อย = มองใกล้ ตอบสนองทันทีแต่เลี้ยวไม่ทัน'),
    _p('mppi_temperature', 'ความเลือกเฉพาะของ MPPI', 0.3, 0.05, 1.0, 0.05, '', section=_NM, live=True,
       desc='ควบคุมว่าจะเฉลี่ยเส้นทางทดลองอย่างไร',
       tip='ต่ำ = ยึดเส้นที่ดีที่สุดเส้นเดียว (ว่องไว) · สูง = เฉลี่ยหลายเส้น (นุ่มแต่เฉื่อย)'),
    _p('w_path_align', 'น้ำหนัก: เกาะแนวเส้นทาง', 14.0, 0.0, 40.0, 1.0, '', section=_NM, live=True,
       desc='ลงโทษเส้นทางทดลองที่เบี่ยงออกนอกแนวเส้นทางที่วางไว้',
       tip='สูง = รถเกาะเส้นแน่น · ต่ำ = รถมีอิสระตัดมุมหรือหลบมากขึ้น'),
    _p('w_path_follow', 'น้ำหนัก: ไปข้างหน้าตามเส้น', 5.0, 0.0, 40.0, 1.0, '', section=_NM, live=True,
       desc='รางวัลให้เส้นทางทดลองที่คืบหน้าไปตามเส้นทางจริง',
       tip='สูง = ไม่อยู่นิ่ง ไปต่อเสมอ'),
    _p('w_goal', 'น้ำหนัก: เข้าหาเป้าหมาย', 5.0, 0.0, 40.0, 1.0, '', section=_NM, live=True,
       desc='ลงโทษเส้นทางที่ห่างจากเป้าหมายเมื่อเข้าใกล้',
       tip='สูง = เข้าเป้าตรงจุด · ต่ำ = ถึงเป้าแบบหลวม ๆ'),
    _p('w_prefer_forward', 'น้ำหนัก: ชอบเดินหน้า', 5.0, 0.0, 40.0, 1.0, '', section=_NM, live=True,
       desc='ลงโทษเส้นทางที่ต้องถอยหลัง',
       tip='สูง = หลีกเลี่ยงการถอย · ต่ำ = ถอยออกจากมุมได้'),
    _p('w_constraint', 'น้ำหนัก: ไม่ฝืนขีดจำกัดรถ', 4.0, 0.0, 40.0, 1.0, '', section=_NM, live=True,
       desc='ลงโทษการสั่งที่เกินความเร็ว/การเลี้ยวที่รถทำได้',
       tip='สูง = สั่งนุ่มนวลอยู่ในขีดจำกัด'),
    _p('controller_frequency', 'ความถี่ตัวควบคุม', 20.0, 5.0, 30.0, 1.0, 'Hz', section=_NM,
       desc='ตัวควบคุมคำนวณคำสั่งใหม่วินาทีละกี่ครั้ง (ต้องรีสตาร์ท Nav2)',
       tip='มาก = ตอบสนองไวแต่ CPU สูง ถ้าคำนวณไม่ทันจะมีคำเตือนใน log'),
    _p('amcl_max_particles', 'จำนวนอนุภาคสูงสุด', 2000, 500, 5000, 100, '', kind='int', section=_NA, live=True,
       desc='AMCL ทดลองเดาตำแหน่งรถด้วยอนุภาคจำนวนมาก แล้วคัดตามที่ตรงกับ lidar',
       tip='มาก = หาตำแหน่งแม่นเมื่อหลงทาง แต่หนัก · น้อย = เบา แต่เสี่ยงหลงตำแหน่ง'),
    _p('amcl_odom_noise', 'ความไม่เชื่อ odometry', 0.2, 0.02, 0.8, 0.02, '', section=_NA, live=True,
       desc='AMCL คิดว่า odometry คลาดเคลื่อนได้เท่าไร (ใช้กับ alpha1–5)',
       tip='สูง = อนุภาคกระจายกว้าง ตามตำแหน่งจริงได้เมื่อล้อลื่น · ต่ำ = เชื่อ odometry มาก (นิ่ง)'),
    _p('amcl_laser_max_range', 'ระยะ lidar ที่ใช้หาตำแหน่ง', 8.0, 2.0, 12.0, 0.5, 'm', section=_NA, live=True,
       desc='ใช้ lidar ไกลสุดเท่านี้เทียบกับแผนที่',
       tip='มาก = มีข้อมูลเทียบมาก แต่ถ้าห้องมีของย้ายที่ไกล ๆ จะรบกวน'),
    _p('amcl_max_beams', 'จำนวนลำแสงที่ใช้ต่อรอบ', 60, 20, 360, 10, '', kind='int', section=_NA, live=True,
       desc='สุ่มใช้ลำแสง lidar เพียงบางส่วนในแต่ละรอบ',
       tip='มาก = แม่นขึ้นแต่ช้า · น้อย = เร็ว'),
]

SCHEMA = {'cartographer': CARTOGRAPHER, 'vslam': VSLAM, 'nav': NAV}
_BY_KEY = {g: {p['key']: p for p in params} for g, params in SCHEMA.items()}

# Where each Nav2 value goes: key -> [(ros node, parameter name, optional function of the value)].
# The same parameter name is used in the YAML (nested at the dots) and in set_parameters.
_LOCAL = 'local_costmap/local_costmap'
_GLOBAL = 'global_costmap/global_costmap'
NAV_TARGETS = {
    'vx_max': [('controller_server', 'FollowPath.vx_max', None)],
    'vx_min': [('controller_server', 'FollowPath.vx_min', None)],
    'wz_max': [('controller_server', 'FollowPath.wz_max', None)],
    'min_turning_r': [('controller_server', 'FollowPath.AckermannConstraints.min_turning_r', None)],
    'xy_goal_tolerance': [('controller_server', 'general_goal_checker.xy_goal_tolerance', None)],
    'yaw_goal_tolerance': [('controller_server', 'general_goal_checker.yaw_goal_tolerance', None)],
    'robot_radius': [(_LOCAL, 'robot_radius', None), (_GLOBAL, 'robot_radius', None)],
    'inflation_radius': [(_LOCAL, 'inflation_layer.inflation_radius', None),
                         (_GLOBAL, 'inflation_layer.inflation_radius', None)],
    'cost_scaling_local': [(_LOCAL, 'inflation_layer.cost_scaling_factor', None)],
    'cost_scaling_global': [(_GLOBAL, 'inflation_layer.cost_scaling_factor', None)],
    'obstacle_range': [(_LOCAL, 'voxel_layer.scan.obstacle_max_range', None),
                       (_LOCAL, 'voxel_layer.scan.raytrace_max_range', lambda v: v + 0.5),
                       (_GLOBAL, 'obstacle_layer.scan.obstacle_max_range', None),
                       (_GLOBAL, 'obstacle_layer.scan.raytrace_max_range', lambda v: v + 0.5)],
    'planner_tolerance': [('planner_server', 'GridBased.tolerance', None)],
    'use_astar': [('planner_server', 'GridBased.use_astar', None)],
    'allow_unknown': [('planner_server', 'GridBased.allow_unknown', None)],
    'mppi_batch': [('controller_server', 'FollowPath.batch_size', None)],
    'mppi_steps': [('controller_server', 'FollowPath.time_steps', None)],
    'mppi_temperature': [('controller_server', 'FollowPath.temperature', None)],
    'w_path_align': [('controller_server', 'FollowPath.PathAlignCritic.cost_weight', None)],
    'w_path_follow': [('controller_server', 'FollowPath.PathFollowCritic.cost_weight', None)],
    'w_goal': [('controller_server', 'FollowPath.GoalCritic.cost_weight', None)],
    'w_prefer_forward': [('controller_server', 'FollowPath.PreferForwardCritic.cost_weight', None)],
    'w_constraint': [('controller_server', 'FollowPath.ConstraintCritic.cost_weight', None)],
    'controller_frequency': [('controller_server', 'controller_frequency', None)],
    'amcl_max_particles': [('amcl', 'max_particles', None)],
    'amcl_odom_noise': [('amcl', f'alpha{i}', None) for i in range(1, 6)],
    'amcl_laser_max_range': [('amcl', 'laser_max_range', None)],
    'amcl_max_beams': [('amcl', 'max_beams', None)],
}

# ---------------------------------------------------------------------------------------
# Presets: a named set of values for a group (applied on top of the defaults).
# ---------------------------------------------------------------------------------------
PRESETS = {
    'cartographer': {
        'standard': {'label': 'มาตรฐาน', 'desc': 'ค่าที่ใช้ได้ดีกับห้องทั่วไป', 'values': {}},
        'precise': {'label': 'ละเอียด', 'desc': 'ช่องกริดเล็ก เก็บสแกนถี่ ต้องขับช้า ๆ',
                    'values': {'resolution': 0.03, 'motion_angle': 0.05, 'motion_distance': 0.1,
                               'submap_scans': 60, 'optimize_every': 45, 'loop_sampling': 0.5}},
        'light': {'label': 'เบา เร็ว', 'desc': 'ใช้ CPU น้อย เหมาะกับพื้นที่ใหญ่หรือรถวิ่งเร็ว',
                  'values': {'resolution': 0.08, 'online_scan_matching': False, 'motion_distance': 0.4,
                             'submap_scans': 120, 'loop_sampling': 0.15}},
        'no_loop': {'label': 'ไม่ปิดลูป (สอน)', 'desc': 'ปิด loop closure เพื่อดูแผนที่เพี้ยนสะสม',
                    'values': {'loop_closure': False}},
    },
    'vslam': {
        'standard': {'label': 'มาตรฐาน', 'desc': 'สมดุลระหว่างความแม่นกับความเร็ว', 'values': {}},
        'precise': {'label': 'ละเอียด', 'desc': 'ภาพใหญ่ feature มาก ต้องขับช้า ๆ',
                    'values': {'image_width': 640, 'max_features': 1000, 'feature_type': 2,
                               'rate': 6.0, 'cell_size': 0.04, 'detection_rate': 1.5}},
        'light': {'label': 'เบา เร็ว', 'desc': 'ภาพเล็ก feature น้อย เหมาะกับ Jetson ที่งานเยอะ',
                  'values': {'image_width': 320, 'max_features': 400, 'feature_type': 6,
                             'rate': 6.0, 'detection_rate': 0.5}},
        'with_lidar': {'label': 'กล้อง + Lidar', 'desc': 'กล้องปิดลูป Lidar วาดผนัง แม่นที่สุด',
                       'values': {'grid_source': 'both'}},
    },
    'nav': {
        'standard': {'label': 'มาตรฐาน', 'desc': 'ค่าตั้งต้นของโปรเจกต์', 'values': {}},
        'gentle': {'label': 'นุ่มนวล (สาธิตใกล้คน)',
                   'desc': 'ช้า เลี้ยวกว้าง เว้นระยะจากผนังเยอะ',
                   'values': {'vx_max': 0.12, 'min_turning_r': 0.5, 'inflation_radius': 0.25,
                              'robot_radius': 0.25, 'xy_goal_tolerance': 0.3, 'yaw_goal_tolerance': 0.5,
                              'cost_scaling_local': 4.0, 'cost_scaling_global': 2.0}},
        'fast': {'label': 'เร็ว',
                 'desc': 'วิ่งเร็วขึ้น ใช้ในที่โล่งเท่านั้น (ระวังเบรกไม่ทัน)',
                 'values': {'vx_max': 0.35, 'min_turning_r': 0.5, 'inflation_radius': 0.2,
                            'robot_radius': 0.25, 'xy_goal_tolerance': 0.35, 'yaw_goal_tolerance': 0.8,
                            'w_path_align': 10.0, 'cost_scaling_local': 3.0, 'cost_scaling_global': 1.5}},
        'tight': {'label': 'ช่องแคบ',
                  'desc': 'ลอดทางแคบได้ ช้าและเข้าใกล้ผนังมากขึ้น',
                  'values': {'vx_max': 0.08, 'min_turning_r': 0.5, 'robot_radius': 0.18,
                             'inflation_radius': 0.08, 'xy_goal_tolerance': 0.15}},
    },
}


# ---------------------------------------------------------------------------------------
# Value handling
# ---------------------------------------------------------------------------------------

def defaults(group):
    return {p['key']: p['default'] for p in SCHEMA[group]}


def coerce(group, key, value):
    """The value as the right type inside the allowed range; ValueError if impossible."""
    spec = _BY_KEY[group].get(key)
    if spec is None:
        raise ValueError(f'unknown parameter {key}')
    kind = spec['kind']
    if kind == 'bool':
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)
    if kind == 'enum':
        for opt_value, _label in spec['options']:
            if str(opt_value) == str(value):
                return opt_value
        raise ValueError(f'{key}: {value!r} is not an option')
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise ValueError(f'{key}: not a number')
    if not math.isfinite(number):
        raise ValueError(f'{key}: not finite')
    number = max(spec['min'], min(spec['max'], number))
    return int(round(number)) if kind == 'int' else round(number, 6)


def clean(group, values):
    """{key: coerced value} for the valid entries of `values`; the rest is dropped."""
    out = {}
    if isinstance(values, dict):
        for key, value in values.items():
            try:
                out[key] = coerce(group, key, value)
            except (ValueError, KeyError):
                pass
    return out


def schema_json():
    """What the page needs to build its sliders."""
    groups = {}
    for group, params in SCHEMA.items():
        groups[group] = {
            'label': GROUP_LABELS[group],
            'params': [{k: v for k, v in p.items()} for p in params],
            'presets': {name: {'label': pr['label'], 'desc': pr['desc'], 'values': pr['values']}
                        for name, pr in PRESETS[group].items()},
        }
    return groups


class Store:
    """The saved overrides of every group (thread-safe, written atomically)."""

    def __init__(self, path=TUNING_PATH):
        self.path = os.path.expanduser(path)
        self._lock = threading.Lock()
        self._data = self._read()

    def _read(self):
        try:
            with open(self.path) as f:
                raw = json.load(f)
        except (OSError, ValueError):
            raw = {}
        return {g: clean(g, raw.get(g)) for g in GROUPS}

    def values(self, group):
        """Every value of the group: the defaults with the saved changes on top."""
        with self._lock:
            return {**defaults(group), **self._data[group]}

    def overrides(self, group):
        with self._lock:
            return dict(self._data[group])

    def update(self, group, values, replace=False):
        """Apply `values` (a dict of changes) and save. replace: forget the other changes."""
        values = clean(group, values)
        with self._lock:
            base = {} if replace else dict(self._data[group])
            base.update(values)
            dflt = defaults(group)
            self._data[group] = {k: v for k, v in base.items() if v != dflt[k]}
            self._write()
            return {**dflt, **self._data[group]}

    def reset(self, group):
        return self.update(group, {}, replace=True)

    def _write(self):
        os.makedirs(os.path.dirname(self.path) or '.', exist_ok=True)
        tmp = self.path + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(self._data, f, indent=2, sort_keys=True)
        os.replace(tmp, self.path)


def load_values(group, path=TUNING_PATH):
    """For the launch files: the current values of a group, read from the file."""
    return Store(path).values(group)


# ---------------------------------------------------------------------------------------
# Cartographer
# ---------------------------------------------------------------------------------------

def _lua(value):
    if isinstance(value, bool):
        return 'true' if value else 'false'
    return repr(float(value)) if isinstance(value, float) else str(value)


def cartographer_lua(base_text, v):
    """The base Lua file with the tuned options set just before `return options`."""
    b2d, pg = 'TRAJECTORY_BUILDER_2D', 'POSE_GRAPH'
    inserter = f'{b2d}.submaps.range_data_inserter.probability_grid_range_data_inserter'
    lines = [
        f'{b2d}.min_range = {_lua(v["min_range"])}',
        f'{b2d}.max_range = {_lua(v["max_range"])}',
        f'{b2d}.missing_data_ray_length = {_lua(v["max_range"])}',
        f'{b2d}.use_imu_data = {_lua(v["use_imu"])}',
        f'{b2d}.use_online_correlative_scan_matching = {_lua(v["online_scan_matching"])}',
        f'{b2d}.real_time_correlative_scan_matcher.linear_search_window = {_lua(v["linear_window"])}',
        f'{b2d}.real_time_correlative_scan_matcher.angular_search_window = math.rad({_lua(v["angular_window"])})',
        f'{b2d}.ceres_scan_matcher.translation_weight = {_lua(v["translation_weight"])}',
        f'{b2d}.ceres_scan_matcher.rotation_weight = {_lua(v["rotation_weight"])}',
        f'{b2d}.motion_filter.max_angle_radians = math.rad({_lua(v["motion_angle"])})',
        f'{b2d}.motion_filter.max_distance_meters = {_lua(v["motion_distance"])}',
        f'{b2d}.submaps.num_range_data = {_lua(v["submap_scans"])}',
        f'{b2d}.submaps.grid_options_2d.resolution = {_lua(v["resolution"])}',
        f'{inserter}.hit_probability = {_lua(v["hit_probability"])}',
        f'{inserter}.miss_probability = {_lua(v["miss_probability"])}',
        f'{pg}.constraint_builder.min_score = {_lua(v["loop_min_score"])}',
        f'{pg}.constraint_builder.global_localization_min_score = {_lua(max(v["loop_min_score"], 0.7))}',
        f'{pg}.constraint_builder.sampling_ratio = {_lua(v["loop_sampling"])}',
        # 0 turns the global optimisation, and so loop closure, off
        f'{pg}.optimize_every_n_nodes = {_lua(v["optimize_every"] if v["loop_closure"] else 0)}',
    ]
    text = base_text.rstrip()
    marker = 'return options'
    head = text[:text.rindex(marker)] if marker in text else text + '\n'
    return head + '-- set by qcar2_web (tuning.py)\n' + '\n'.join(lines) + '\n\n' + marker + '\n'


# ---------------------------------------------------------------------------------------
# V-SLAM (RTAB-Map)
# ---------------------------------------------------------------------------------------

def rtabmap_parameters(v, localization=False):
    """(ROS parameters, RTAB-Map parameters as strings) for the rtabmap node."""
    sensor = {'depth': 1, 'lidar': 0, 'both': 2}[v['grid_source']]
    ros = {
        'use_sim_time': False,
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',      # wheel + gyro odometry from qcar2_odom
        'map_frame_id': 'map',
        'publish_tf': True,
        'subscribe_rgb': True,
        'subscribe_depth': True,
        'subscribe_scan': sensor != 1,
        'approx_sync': True,
        'queue_size': 20,
        'wait_for_transform': 0.3,
        'odom_tf_linear_variance': 0.01,
        'odom_tf_angular_variance': 0.01,
        'database_path': os.path.expanduser(VSLAM_LOC_DB_PATH if localization else VSLAM_DB_PATH),
    }
    r = {
        'Rtabmap/DetectionRate': v['detection_rate'],
        'Rtabmap/LoopThr': v['loop_threshold'],
        'RGBD/LinearUpdate': v['linear_update'],
        'RGBD/AngularUpdate': round(math.radians(v['angular_update']), 4),
        'RGBD/OptimizeMaxError': v['optimize_max_error'],
        'Vis/MinInliers': v['min_inliers'],
        'Vis/MaxFeatures': v['max_features'],
        'Vis/FeatureType': v['feature_type'],
        'Vis/MaxDepth': v['feature_max_depth'],
        'Kp/MaxFeatures': v['max_features'],
        'Kp/DetectorStrategy': v['feature_type'],
        'Kp/MaxDepth': v['feature_max_depth'],
        'Reg/Strategy': 0,            # visual registration; lidar only draws the grid
        'Reg/Force3DoF': True,        # the car stays on the floor
        'Optimizer/Slam2D': True,
        'Grid/Sensor': sensor,
        'Grid/3D': False,
        'Grid/CellSize': v['cell_size'],
        'Grid/RangeMin': 0.15,
        'Grid/RangeMax': v['depth_range'],
        'Grid/MaxGroundHeight': v['ground_height'],
        'Grid/MaxObstacleHeight': v['obstacle_height'],
        'Grid/NormalsSegmentation': False,   # plain height test; the floor is flat
        'Grid/DepthDecimation': 2,
        'Grid/FootprintRadius': 0.12,
        'Grid/RayTracing': v['ray_tracing'],
        'Grid/NoiseFilteringRadius': v['noise_radius'],
        'Grid/NoiseFilteringMinNeighbors': 4,
        'Mem/IncrementalMemory': not localization,
        'Mem/InitWMWithAllNodes': localization,
    }
    return ros, {k: (str(val).lower() if isinstance(val, bool) else str(val)) for k, val in r.items()}


def _quat_from_matrix(m):
    """(x, y, z, w) of a 3x3 rotation matrix given as nested lists."""
    t = m[0][0] + m[1][1] + m[2][2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, 0.25 * s)
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        return (0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s)
    if m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        return ((m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s)
    s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
    return ((m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s, (m[1][0] - m[0][1]) / s)


def camera_transform(obstacle_cfg):
    """base_link -> RealSense optical frame as (x, y, z, qx, qy, qz, qw).

    Uses the camera mounting saved by the obstacle page (obstacle.camera_to_vehicle, whose
    frame has its origin under the middle of the car), so one calibration serves both.
    base_link is on the rear axle, wheelbase/2 behind that origin.
    """
    from qcar2_web import obstacle
    f = lambda a, b, c: obstacle.camera_to_vehicle(obstacle_cfg, a, b, c)   # noqa: E731
    o = f(0, 0, 0)
    cols = [tuple(p - q for p, q in zip(f(*e), o)) for e in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]
    matrix = [[cols[c][r] for c in range(3)] for r in range(3)]
    qx, qy, qz, qw = _quat_from_matrix(matrix)
    return (o[0] + obstacle_cfg['wheelbase'] / 2.0, o[1], o[2], qx, qy, qz, qw)


# ---------------------------------------------------------------------------------------
# Nav2
# ---------------------------------------------------------------------------------------

def _yaml_path(node, name):
    parts = node.split('/')
    return parts + ['ros__parameters'] + name.split('.')


def _set_path(tree, path, value):
    for part in path[:-1]:
        nxt = tree.get(part)
        if not isinstance(nxt, dict):
            nxt = tree[part] = {}
        tree = nxt
    tree[path[-1]] = value


def nav2_parameters(base, values):
    """A copy of the Nav2 parameter tree with the tuned values written in."""
    tree = copy.deepcopy(base)
    for key, value in values.items():
        for node, name, fn in NAV_TARGETS.get(key, ()):
            _set_path(tree, _yaml_path(node, name), fn(value) if fn else value)
    return tree


def write_nav2_params(base_path, out_path, values=None):
    """Write base_path + the saved Nav2 tuning to out_path; returns out_path."""
    with open(base_path) as f:
        base = yaml.safe_load(f)
    tree = nav2_parameters(base, load_values('nav') if values is None else values)
    out_path = os.path.expanduser(out_path)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, 'w') as f:
        yaml.safe_dump(tree, f, sort_keys=False)
    return out_path


def nav_live_updates(changed):
    """{ros node: [(parameter name, value), ...]} to send to a running Nav2."""
    out = {}
    for key, value in changed.items():
        spec = _BY_KEY['nav'].get(key)
        if spec is None or not spec['live']:
            continue
        for node, name, fn in NAV_TARGETS.get(key, ()):
            out.setdefault(node, []).append((name, fn(value) if fn else value))
    return out

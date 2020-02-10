# This script is to run all the experiments in one program

import os
import subprocess
import time
import signal

# SeqNameList = ['MH_03_medium'];
SeqNameList = ['MH_01_easy', 'V2_02_medium', 'MH_05_difficult'];
# SeqNameList = ['MH_01_easy', 'MH_02_easy', 'MH_03_medium', 'MH_04_difficult', 'MH_05_difficult', 'V1_01_easy', 'V1_02_medium', 'V1_03_difficult', 'V2_01_easy', 'V2_02_medium', 'V2_03_difficult'];

Result_root = '/mnt/DATA/tmp/EuRoC/msckf_Stereo_Speedx1/'
# Result_root = '/mnt/DATA/tmp/EuRoC/msckf_Stereo_Speedx2/'
# Result_root = '/mnt/DATA/tmp/EuRoC/msckf_Stereo_Speedx4/'

rate = 1.0

Number_GF_List = [200] # [70, 150, 200, 400]; 

Num_Repeating = 10 # 20 # 1 # 
SleepTime = 5

#----------------------------------------------------------------------------------------------------------------------
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    ALERT = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

for ri, num_gf in enumerate(Number_GF_List):
    
    Experiment_prefix = 'ObsNumber_' + str(int(num_gf))

    for iteration in range(0, Num_Repeating):

        Experiment_dir = Result_root + Experiment_prefix + '_Round' + str(iteration + 1)
        cmd_mkdir = 'mkdir -p ' + Experiment_dir
        subprocess.call(cmd_mkdir, shell=True)

        for sn, sname in enumerate(SeqNameList):
            
            print bcolors.ALERT + "====================================================================" + bcolors.ENDC

            SeqName = SeqNameList[sn] #+ '_blur_9'
            print bcolors.ALERT + "Round: " + str(iteration + 1) + "; Seq: " + SeqName

            File_rosbag  = '/mnt/DATA/Datasets/EuRoC_dataset/BagFiles/' + SeqName + '.bag'

            cmd_slam   = str('roslaunch msckf_vio msckf_vio_euroc.launch track_per_frame:=') + str(num_gf)
            cmd_rosbag = 'rosbag play ' + File_rosbag + ' -r ' + str(rate) # + ' -u 30' # 
            cmd_lmklog = str('cp /mnt/DATA/msckf_tmpLog_lmk.txt ' + Experiment_dir + '/' + SeqName + '_Log_lmk.txt')
            cmd_timelog_1 = str('cp /mnt/DATA/msckf_tmpLog_front.txt ' + Experiment_dir + '/' + SeqName + '_Log_front.txt')
            cmd_timelog_2 = str('cp /mnt/DATA/msckf_tmpLog_back.txt ' + Experiment_dir + '/' + SeqName + '_Log_back.txt')
            cmd_tracklog = str('cp /mnt/DATA/msckf_tmpTrack.txt ' + Experiment_dir + '/' + SeqName + '_AllFrameTrajectory.txt')

            print bcolors.WARNING + "cmd_slam: \n"   + cmd_slam   + bcolors.ENDC
            print bcolors.WARNING + "cmd_rosbag: \n" + cmd_rosbag + bcolors.ENDC
            print bcolors.WARNING + "cmd_lmklog: \n" + cmd_lmklog + bcolors.ENDC
            print bcolors.WARNING + "cmd_timelog_1: \n" + cmd_timelog_1 + bcolors.ENDC
            print bcolors.WARNING + "cmd_timelog_2: \n" + cmd_timelog_2 + bcolors.ENDC
            print bcolors.WARNING + "cmd_tracklog: \n" + cmd_tracklog + bcolors.ENDC

            print bcolors.OKGREEN + "Launching SLAM" + bcolors.ENDC
            proc_slam = subprocess.Popen(cmd_slam, shell=True)
            # proc_slam = subprocess.Popen("exec " + cmd_slam, stdout=subprocess.PIPE, shell=True)

            print bcolors.OKGREEN + "Sleeping for a few secs to wait for msckf_vio init" + bcolors.ENDC
            time.sleep(SleepTime)

            print bcolors.OKGREEN + "Launching rosbag" + bcolors.ENDC
            proc_bag = subprocess.call(cmd_rosbag, shell=True)

            print bcolors.OKGREEN + "Finished rosbag playback, kill the process" + bcolors.ENDC
            subprocess.call('rosnode kill /firefly_sbx/image_processor', shell=True)
            subprocess.call('rosnode kill /firefly_sbx/vio', shell=True)
            # subprocess.call('pkill roslaunch', shell=True)
            # subprocess.call('pkill svo_node', shell=True)

            print bcolors.OKGREEN + "Sleeping for a few secs to wait for msckf_vio quit" + bcolors.ENDC
            time.sleep(SleepTime)
            print bcolors.OKGREEN + "Copy the lmk log to result folder" + bcolors.ENDC
            subprocess.call(cmd_lmklog, shell=True)
            print bcolors.OKGREEN + "Copy the time log to result folder" + bcolors.ENDC
            subprocess.call(cmd_timelog_1, shell=True)
            subprocess.call(cmd_timelog_2, shell=True)
            print bcolors.OKGREEN + "Copy the track to result folder" + bcolors.ENDC
            subprocess.call(cmd_tracklog, shell=True)
            # proc_rec.terminate()
            # outs, errs = proc_rec.communicate()
            # proc_slam.kill()

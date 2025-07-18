#!/bin/bash

Green_font_prefix="\033[32m" && Red_font_prefix="\033[31m" && Green_background_prefix="\033[42;37m" && Red_background_prefix="\033[41;37m" && Yellow_background_prefix="\033[43;37m" && Font_color_suffix="\033[0m" && Yellow_font_prefix="\e[1;33m" && Blue_font_prefix="\e[0;34m"
Info="${Green_font_prefix}[信息]${Font_color_suffix}"
echo -e "${Info} 这个是在 ROS 层面上地点记录的脚本, 仅仅用于 debug"
echo -e "${Info} 脚本运行过程任意时候可按 Ctrl+C 退出脚本"
echo -e ""
echo -e "${Info} 接下来请根据提示进行操作"

# echo -e ""
# echo -e "${Info} 控制 spark 移动到 ${Yellow_background_prefix}收取区A${Font_color_suffix}\c"
# echo && stty erase '^H' && read -p "       然后在这个界面内按回车..." 
# rostopic pub /mark_nav std_msgs/String "data: 'learn Collection_A'" -1 >/dev/null
# echo -e "${Info} 地点名为 ${Yellow_background_prefix}sp${Font_color_suffix}"

echo -e ""
echo -e "${Info} 控制 spark 移动到 ${Yellow_background_prefix}分类区${Font_color_suffix}\c"
echo && stty erase '^H' && read -p "       然后在这个界面内按回车..." 
rostopic pub /mark_nav std_msgs/String "data: 'learn Classification_area'" -1 >/dev/null
echo -e "${Info} 地点名为 ${Yellow_background_prefix}Classification_area${Font_color_suffix}"

echo -e ""
echo -e "${Info} 控制 spark 移动到 ${Yellow_background_prefix}分类区${Font_color_suffix}\c"
echo && stty erase '^H' && read -p "       然后在这个界面内按回车..." 
rostopic pub /mark_nav std_msgs/String "data: 'learn Classification_area'" -1 >/dev/null
echo -e "${Info} 地点名为 ${Yellow_background_prefix}Classification_area${Font_color_suffix}"


echo -e ""
echo -e "${Info} 控制 spark 移动到 ${Yellow_background_prefix}收取区B${Font_color_suffix}\c"
echo && stty erase '^H' && read -p "       然后在这个界面内按回车..." 
rostopic pub /mark_nav std_msgs/String "data: 'learn Collection_B'" -1 >/dev/null
echo -e "${Info} 地点名为 ${Yellow_background_prefix}Collection_B${Font_color_suffix}"

echo -e ""
echo -e "${Info} 控制 spark 移动到 ${Yellow_background_prefix}收取区C${Font_color_suffix}\c"
echo && stty erase '^H' && read -p "       然后在这个界面内按回车..." 
rostopic pub /mark_nav std_msgs/String "data: 'learn Collection_C'" -1 >/dev/null
echo -e "${Info} 地点名为 ${Yellow_background_prefix}Collection_C${Font_color_suffix}"

echo -e ""
echo -e "${Info} 控制 spark 移动到 ${Yellow_background_prefix}收取区D${Font_color_suffix}\c"
echo && stty erase '^H' && read -p "       然后在这个界面内按回车..." 
rostopic pub /mark_nav std_msgs/String "data: 'learn Collection_D'" -1 >/dev/null
echo -e "${Info} 地点名为 ${Yellow_background_prefix}Collection_D${Font_color_suffix}"

echo -e ""
echo -e "${Info} 控制 spark 移动到 ${Yellow_background_prefix}分拣区东${Font_color_suffix}\c"
echo && stty erase '^H' && read -p "       然后在这个界面内按回车..." 
rostopic pub /mark_nav std_msgs/String "data: 'learn Sorting_E'" -1 >/dev/null
echo -e "${Info} 地点名为 ${Yellow_background_prefix}Sorting_E${Font_color_suffix}"

echo -e ""
echo -e "${Info} 控制 spark 移动到 ${Yellow_background_prefix}分拣区北${Font_color_suffix}\c"
echo && stty erase '^H' && read -p "       然后在这个界面内按回车..." 
rostopic pub /mark_nav std_msgs/String "data: 'learn Sorting_N'" -1 >/dev/null
echo -e "${Info} 地点名为 ${Yellow_background_prefix}Sorting_N${Font_color_suffix}"

echo -e ""
echo -e "${Info} 控制 spark 移动到 ${Yellow_background_prefix}分拣区西${Font_color_suffix}\c"
echo && stty erase '^H' && read -p "       然后在这个界面内按回车..." 
rostopic pub /mark_nav std_msgs/String "data: 'learn Sorting_W'" -1 >/dev/null
echo -e "${Info} 地点名为 ${Yellow_background_prefix}Sorting_W${Font_color_suffix}"

echo -e ""
echo -e "${Info} 控制 spark 移动到 ${Yellow_background_prefix}分拣区南${Font_color_suffix}\c"
echo && stty erase '^H' && read -p "       然后在这个界面内按回车..." 
rostopic pub /mark_nav std_msgs/String "data: 'learn Sorting_S'" -1 >/dev/null
echo -e "${Info} 地点名为 ${Yellow_background_prefix}Sorting_S${Font_color_suffix}"


echo -e ""
echo -e "${Info} 是否标记完成,保存地图？ \c"
echo && stty erase '^H' && read -p "       然后在这个界面内按回车..." 
rostopic pub /mark_nav std_msgs/String "data: 'finish'" -1 >/dev/null
# 保存地图
CURRENTPATH=$(cd `dirname $0`; pwd)
gnome-terminal -x bash -c "rosrun map_server map_saver -f $CURRENTPATH/../config/test_map"
echo -e "${Info} 结束标记地点,已保存地图"

echo -e ""
echo -e "${Info} 是否启动比赛程序？ \c"
echo && stty erase '^H' && read -p "       然后在这个界面内按回车..." 
rostopic pub /task_start_flag std_msgs/String "data: 'true'" -1 >/dev/null
echo -e "${Info} 开始比赛"

# echo -e ""
# echo -e "${Info} 控制 spark 移动到 ${Yellow_background_prefix}收取区D${Font_color_suffix}\c"
# echo && stty erase '^H' && read -p "       然后在这个界面内按回车..." 
# rostopic pub /mark_nav std_msgs/String "data: 'learn Collection_D'" -1 >/dev/null
# echo -e "${Info} 地点名为 ${Yellow_background_prefix}wp_1${Font_color_suffix}"

# echo -e ""
# echo -e "${Info} 控制 spark 移动到 ${Yellow_background_prefix}收取区C${Font_color_suffix}\c"
# echo && stty erase '^H' && read -p "       然后在这个界面内按回车..." 
# rostopic pub /mark_nav std_msgs/String "data: 'learn Collection_C'" -1 >/dev/null
# echo -e "${Info} 地点名为 ${Yellow_background_prefix}wp_1${Font_color_suffix}"

# echo -e ""
# echo -e "${Info} 控制 spark 移动到 ${Yellow_background_prefix}收取区A${Font_color_suffix}\c"
# echo && stty erase '^H' && read -p "       然后在这个界面内按回车..." 
# rostopic pub /mark_nav std_msgs/String "data: 'learn Collection_A'" -1 >/dev/null
# echo -e "${Info} 地点名为 ${Yellow_background_prefix}wp_1${Font_color_suffix}"

# echo -e ""
# echo -e "${Info} 控制 spark 移动到 ${Yellow_background_prefix}分拣区北${Font_color_suffix}\c"
# echo && stty erase '^H' && read -p "       然后在这个界面内按回车..." 
# rostopic pub /mark_nav std_msgs/String "data: 'learn Sorting_N'" -1 >/dev/null
# echo -e "${Info} 地点名为 ${Yellow_background_prefix}wp_1${Font_color_suffix}"

# echo -e ""
# echo -e "${Info} 控制 spark 移动到 ${Yellow_background_prefix}分拣区西${Font_color_suffix}\c"
# echo && stty erase '^H' && read -p "       然后在这个界面内按回车..." 
# rostopic pub /mark_nav std_msgs/String "data: 'learn Sorting_W'" -1 >/dev/null
# echo -e "${Info} 地点名为 ${Yellow_background_prefix}wp_1${Font_color_suffix}"

# echo -e ""
# echo -e "${Info} 控制 spark 移动到 ${Yellow_background_prefix}分拣区南${Font_color_suffix}\c"
# echo && stty erase '^H' && read -p "       然后在这个界面内按回车..." 
# rostopic pub /mark_nav std_msgs/String "data: 'learn Sorting_S'" -1 >/dev/null
# echo -e "${Info} 地点名为 ${Yellow_background_prefix}wp_1${Font_color_suffix}"

# echo -e ""
# echo -e "${Info} 控制 spark 移动到 ${Yellow_background_prefix}分拣区东${Font_color_suffix}\c"
# echo && stty erase '^H' && read -p "       然后在这个界面内按回车..." 
# rostopic pub /mark_nav std_msgs/String "data: 'learn Sorting_E'" -1 >/dev/null
# echo -e "${Info} 地点名为 ${Yellow_background_prefix}wp_1${Font_color_suffix}"


echo -e ""
echo -e "${Info} 地点记录已完成\c"
echo -e ""
echo && stty erase '^H' && read -p "现在可以按回车退出脚本..." 

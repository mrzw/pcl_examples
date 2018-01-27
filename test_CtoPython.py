# coding: utf8
from ctypes import *
# 调用函数前加上extern "C" (python只支持c风格), 下面是其中一个实例
########C++部分#########################
# extern "C"
# double* icp(char str1[],char str2[],double* trajectory)   // str1, str2是个字符串, trajectory是个一维向量
# {
#   ...
#   double* result = new double[16];
#   ...
#   return result;
# }
#######################################
def geticp(rgbfile, depthfile, pose):
    lib = cdll.LoadLibrary('××.so')
    func = lib.icp                          # icp是C++程序对应的函数
    func.restype = c_double                 # 设置返回值类型，默认输出int型
    func.restype = POINTER(c_double * 16)   # 当返回是个array(这里以输出16维度为例), 获取返回值: ans[0][0] ... ans[0][15]
    pose = (c_double * len(pose))(*pose)    # 当输入是个list表的浮点数
    rgbfile = rgbfile.encode('utf-8')       # 传入的字符串参数必须为encode('utf-8')，否则在c库中仅会打印为首字符
    depthfile = depthfile.encode('utf-8')
    ans = func(rgbfile, depthfile, pose)

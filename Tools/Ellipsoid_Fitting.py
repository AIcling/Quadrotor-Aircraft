import torch
import math
data_scale =2
data_list = []

with open("data.txt","r") as f:
    for i in range(data_scale):
        data = f.readline().replace('\r','').replace('\n','')
        x,y,z = data.split(" ")
        mx = float(x)
        my = float(y)
        mz = float(z)
        data_list.append([])
        data_list[i].append(mx)
        data_list[i].append(my)
        data_list[i].append(mz)  
f.close()
Ox=Oy=Oz=Rx=Ry=Rz=Ox_sum=Oy_sum=Oz_sum=Rx_sum=Ry_sum=Rz_sum=float(0)
for i in range(data_scale):
    mx = data_list[i][0]
    my = data_list[i][1]
    mz = data_list[i][2]
    k = torch.FloatTensor([[my*my,mz*mz,mx,my,mz,1]])
    k_t = k.t()
    tmp = torch.matmul(k_t,k)
    tmp = torch.linalg.pinv(tmp)
    tmp = torch.matmul(tmp,k_t)
    #print(tmp.size())
    res = tmp*-mx*mx
    print(res)
    #print(res.size())
    Ox = res[2][0].float()/-2
    Oy = res[3][0].float()/-2*res[0][0]
    Oz = res[4][0].float()/-2*res[1][0]
    Rx = math.sqrt(Ox*Ox+res[0][0]*Oy*Oy+res[1][0]*Oz*Oz-res[5][0])
    Ry = math.sqrt(Rx*Rx/abs(res[0][0]))
    Rz = math.sqrt(Rx*Rx/abs(res[1][0]))
    Rx_sum+=Rx
    Ry_sum+=Ry
    Rz_sum+=Rz
    Ox_sum+=Ox
    Oy_sum+=Oy
    Oz_sum+=Oz

print(Rx_sum/data_scale,Ry_sum/data_scale,Rz_sum/data_scale,Ox_sum/data_scale,Oy_sum/data_scale,Oz_sum/data_scale)
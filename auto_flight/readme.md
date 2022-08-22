# NCRL LINK

## 

buf|0| 1| 2| 3|4~6|7|8
---|---|---|---|--|--|--|-
data|start byte| checksum|mode |aux_info|pos|-|end byte

## send data to pc 

topic : /pc_to_pixhawk 
msg form : auto_flight/ncrl_link.msg (you need to source this pakege if you want to use this topic.)


task | mode | data1 ~ data3 |  
--|--|--
default | 0 | -
takeoff | 1 | -
waypoint | 2 |waypoint xyz data in enu [m]
landing | 3 | -

example :
```
rostopic pub /pc_to_pixhawk auto_flight/ncrl_link "mode: '0', aux_info: '', data1: 0.0, data2: 0.0, data3: 0.0}"
```





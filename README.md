# rosbridge_client
## 安裝
```
pip install roslibpy
```

接收 jetson 上面的 /out/compressed ， 然後透過這個 rosbridge client 端轉發到本地端

有兩個參數，分別是
- --rosbridge_ip
- --rosbridge_port (這個預設為 9090)
  
使用範例 :
```
python3 rosbridge_client.py --rosbridge_ip 192.168.0.207 --rosbridge_port 9090
```
or 
```
python3 rosbridge_client.py --rosbridge_ip 192.168.0.207
```

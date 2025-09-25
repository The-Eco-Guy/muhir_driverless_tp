
#part 1
data=[]
dictdata={}
with open("test.csv", "r") as file:
    block=file.read()
    data=block.split('\n')
data.pop()
print(data)
for i in data:
    j=i.split(',')
    dictdata[j[0]]=j[1]
dictdata=dict(sorted(dictdata.items()))
print(dictdata)
with open("test1.csv","w+") as file:
    for k,v in dictdata.items():
        file.write(k+","+v+"\n")

#part 2
with open("test1.csv","r") as file:
    block=file.read()
    data=block.split('\n')
    data.pop()
data2=data[::2]
print(data2)
with open("test2.csv","w") as file:
    for i in data2:
        file.write(i+"\n")

#part 3
data3=""
for i in data:
    j=i.split(',')
    data3+=j[1]
print("data 3 is ",data3)
mindiff=ord((data3[1]))-ord((data3[0]))
key1=0
key2=0
# if difference has to be computed between adjacant elements
for i in range(2,len(data3)):
    if(abs(ord(data3[i])-ord(data3[i-1]))<mindiff):
        mindiff=abs(ord(data3[i])-ord(data3[i-1]))
        key1=i
        key2=i-1
print(f"minimum absolute ascii difference is {mindiff} and is present between indexes {key1} and {key2} which are letters {data3[18]} and {data3[17]}")

#if difference has to be computed between overall elements
flag=1
data3list=[]
for i in data3:
    data3list.append(i)
for i in range(0,len(data3list)-1):
    smallest=i
    for j in range(i,len(data3list)):
        if (data3list[j]==data3list[smallest]):
            flag=0
            break

        if (data3list[j]<data3list[smallest]):
            smallest=j
    x=data3list[i]
    data3list[i]=data3list[smallest]
    data3list[smallest]=x
if(flag):
    print(f"sorted data is {data3list}, minimum absolute difference is {ord(data3list[1])-ord(data3list[0])}")
else:
    print(f"minimum absolute difference is {0}")



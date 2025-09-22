class binary:
    def selection_sort(self,data):
        for i in range(0,len(data)-1):
            smallest=i
            for j in range(i+1,len(data)):
                if data[j]<data[smallest]:
                    smallest=j
            x=data[i]
            data[i]=data[smallest]
            data[smallest]=x
    def binary_search(self,data,key):
        low=0
        high=len(data)-1
        while(low<=high):
            mid=int((low+high)/2)
            if (data[mid]==key):
                print("key found")
                return mid
            if (data[mid]<key):
                low=mid+1
            if (data[mid]>key):
                high=mid-1
        return -1


    pass

n=int(input("enter the number of strings: "))
strings=[]
for i in range(0,n):
    x=input("enter string: ")
    x=x.lower()
    strings.append(x)
bin=binary()
bin.selection_sort(strings)
print("sorted strings are",strings)
key=input("enter key to be searched: ")
n1=bin.binary_search(strings,key)
if(n1!=-1):
    print("key found at position ",n1)
else:
    print("key not present")


class selection_sort:
    def selection_sort(self,data):
        for i in range(0,len(data)-1):
            smallest=i
            for j in range(i+1,len(data)):
                if data[j]<data[smallest]:
                    smallest=j
            x=data[i]
            data[i]=data[smallest]
            data[smallest]=x


    pass
string_sorter=selection_sort()
n=int(input("enter the number of strings n: "))
strings=[]
for i in range(0,n):
    x=input("enter string: ")
    x=x.lower()
    strings.append(x)
string_sorter.selection_sort(strings)
print(strings)
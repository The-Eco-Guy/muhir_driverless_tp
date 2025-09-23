
n=int(input("enter the number of integers: "))
loi=[] #list of integers
for i in range(0,n):
    x=int(input(f"enter the {i+1} number: "))
    loi.append(x)
hash_table=[[]*n for i in range(10)]
def hash_function(value):
    x=value%10
    return x


def add(value):
    index=hash_function(value)
    if(hash_table[index]==[]):
        hash_table[index].append(value)
    else:
        for i in range(len(hash_table[index])-1,-1,-1):
            if(i==0):
                hash_table[index].insert(i,value)
                return
            if((value>hash_table[index][i])):
                hash_table[index].insert(i+1,value)

for i in loi:
    add(i)
print(hash_table)


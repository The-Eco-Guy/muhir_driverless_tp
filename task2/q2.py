
n=int(input("enter the number of integers: "))
loi=[] #list of integers
for i in range(0,n):
    x=int(input(f"enter the {i+1} number: "))
    loi.append(x)
hash_table=[[] for i in range(10)]
def hash_function(value):
    x=value%10
    return x

def add(value):
    index=hash_function(value)
    hash_table[index].append(value)
for i in loi:
    add(i)
print(hash_table)
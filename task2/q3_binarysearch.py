n = int(input("enter the number of integers: "))
loi = []
for i in range(0, n):
    x = int(input(f"enter the {i + 1} number: "))
    loi.append(x)

hash_table = [[] for i in range(10)]

def hash_function(value):
    x = value % 10
    return x

def find_insertion_point(arr, target):
    low = 0
    high = len(arr) - 1

    while low <= high:
        mid = low + (high - low) // 2
        if arr[mid] == target:
            return mid
        elif  mid - 1 >= 0 and arr[mid] > target and arr[mid - 1] < target:
            return mid
        elif mid+1>=len(arr) and arr[mid]<target:
            return (mid+1)
        elif mid-1<0 and arr[mid]>target:
            return 0
        elif arr[mid] < target:
            low = mid + 1
        else:
            high = mid - 1
    return low

def add(value):
    index = hash_function(value)
    if hash_table[index] == []:
        hash_table[index].append(value)
        print(hash_table)
    else:
        index2 = find_insertion_point(hash_table[index], value)
        hash_table[index].insert(index2, value)
        print(hash_table)

for i in loi:
    add(i)

print(hash_table)

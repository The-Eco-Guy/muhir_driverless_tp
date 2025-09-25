sample_coordinates = [(5, 5), (4, 4), (3, 3), (2, 2), (1, 1)]
x=int(input("enter the co-ordinate at x axis: "))
y=int(input("enter the co-ordinate at y axis: "))
rp=(x,y) #reference point
def calculate_distance(a,b):
    distance=(abs((b[0]-a[0])**2 + (b[1]-a[1])**2))**0.5
    print(a,b,distance)
    return distance
def sorter():
    for i in range(0,len(sample_coordinates)-1):
        smallest_index=i
        for j in range(i+1,len(sample_coordinates)):
            if((calculate_distance(sample_coordinates[j],rp))<(calculate_distance(sample_coordinates[smallest_index],rp))):
                smallest_index=j
        x=sample_coordinates[i]
        sample_coordinates[i]=sample_coordinates[smallest_index]
        sample_coordinates[smallest_index]=x
    print(sample_coordinates)
sorter()
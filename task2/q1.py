import numpy as np
m1=int(input("enter the number of rows of matrix 1: "))
n1=int(input("enter the number of columns of matrix 1: "))
m2=int(input("enter the number of rows of matrix 2: "))
n2=int(input("enter the number of columns of matrix 2: "))
matrix1=np.zeros((m1,n1))
matrix2=np.zeros((m2,n2))
matrix3=np.zeros((m1,n2))
for i in range(0,m1):
	for j in range(0,n1):
		x=int(input(f" enter value for the {i+1}th row and {j+1}th column for matrix 1: "))
		matrix1[i][j]=x
for i in range(0,m2):
	for j in range(0,n2):
		x=int(input(f" enter value for the {i+1}th row and {j+1}th column for matrix 2: "))
		matrix2[i][j]=x

def multiplymat(a,b):
	sum=0
	if (a.shape[1]!=b.shape[0]):
		print("matrices can't be multiplied as dimensions are invalid")
		return
	for i in range(0,a.shape[0]):
		for j in range(0,b.shape[1]):
			for k in range(0,b.shape[0]):
				sum=sum+((a[i][k])*(b[k][j]))
			matrix3[i][j]=sum
			sum=0
	for i in range(0,a.shape[0]):
		for j in range(0,b.shape[1]):
			print(matrix3[i][j],' ',end='')
		print("\n")
multiplymat(matrix1,matrix2)
	
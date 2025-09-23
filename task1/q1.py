n=int(input("enter the number of strings n: "))
strings=[]
dictionary={}
all_lowercase_letters = "abcdefghijklmnopqrstuvwxyz"
for i in range(0,n):
    x=input("enter string: ")
    x=x.lower()
    strings.append(x)
for i in all_lowercase_letters:
    dictionary[i]=0
for j in strings:
    for i in j:
        dictionary[i]=1+dictionary[i]
print(dictionary)
            

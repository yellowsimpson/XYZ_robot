def add(a,b):
      print(a+b)
def sub(a,b): 
      print(a-b)
def mul(a,b): 
      print(a*b)
def div(a,b):
      print(a/b)
def exp(a,b):
      print(a**b)
def floordiv(a,b):
      print(a//b)
if __name__ == "__main__":
      import sys
      if int(sys.argv[1])==1:
            add(int(sys.argv[2]),int(sys.argv[3]))
      elif int(sys.argv[1])==2:
            sub(int(sys.argv[2]),int(sys.argv[3]))
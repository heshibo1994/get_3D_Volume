l1 = [-0.0199446, -0.390857, -0.243998] 
l1 = [-0.000278531,-0.437374,-0.201781]
l2 = [-0.0318401,-0.383366,-0.185654]
f = open("plane.txt")
lines = f.readlines()
totala = 0
totalb = 0

for line in lines[1:]:
	a = float(line.split(" ")[0])*l1[0]+float(line.split(" ")[1])*l1[1]+float(line.split(" ")[2])*l1[2]+1	
	print(a)
	b = float(line.split(" ")[0])*l2[0]+float(line.split(" ")[1])*l2[1]+float(line.split(" ")[2])*l2[2]+1	
	totala = totala+abs(a)
	totalb = totalb+abs(b)
print(totala,totalb)



k = [[1,1],[1,2],[1,3],[2,3],[2,2]]

d=None
newPath=[]
for pose in k:
    if not newPath:
        newPath.append(pose)
    else:
        if d is None:
            if newPath[-1][0]==pose[0]:
                d=0
            else:
                d=1
        if newPath[-1][d]!=pose[d]:
            newPath.append(pose)
            d=None

if newPath[-1] != k[-1]:
   newPath.append(k[-1])

print(newPath)

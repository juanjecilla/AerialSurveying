function graph = order(points)

graph=[];
pointsXY = [];

for i = 1:size(points,1)
    for j=1:size(points,2)
        pointsXY=[points{i,j}(1) points{i,j}(2)]
        graph=[graph;pointsXY]
    end
end
end
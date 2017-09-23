function graph = generateGraph(points)

graph = [];
aux = [];

for i = 1:size(points,1)
    for j = 1:size(points,2)
        aux = []
        for k = 1:size(points, 1)
            for l = 1:size(points, 2)
               %graph(i*j,i*j) = calcDistance(points{i,j}, points{k,l});
               aux = [aux calcDistance(points{i,j}, points{k,l})];
            end
        end
        
        graph = [graph; aux]
            
    end
    
end


end


% Function to extract path from parent array, start and goal
function extractpath(parent,start,goal)
    temp = parent(goal(1),goal(2));
    path = [];
    current = temp;

    while (true)
        path = [path,current];
        current = parent(current(0),current(1));
%         
%         if current[0]==start[0] and current[1]==start[1]:
%             break
%         
%     path.append(start)
%     path.reverse()
%     path.append(goal)
%        
%     return path
        



    

end





% def extractpath(parent,start,goal):
%     temp = parent[goal[0]][goal[1]]
%     path = []
%     current = temp
%     while True:
%         path.append(current)
%         current = parent[current[0]][current[1]]
%         
%         if current[0]==start[0] and current[1]==start[1]:
%             break
%         
%     path.append(start)
%     path.reverse()
%     path.append(goal)
%        
%     return path



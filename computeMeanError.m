%For each allowable triangle find the point lying in it and compute the mean error
function [meanError]= computeMeanError(Sx, Sy, tBoundaries, error)

meanError=[];

for i=1:2:size(tBoundaries, 2)-1
    if all(~isnan(tBoundaries(:,i))) && all(~isnan(tBoundaries(:,i+1)))
        minX= min([Sx tBoundaries(1,i) tBoundaries(1,i+1)]);
        maxX= max([Sx tBoundaries(1,i) tBoundaries(1,i+1)]);
        minY= min([Sy tBoundaries(2,i) tBoundaries(2,i+1)]);
        maxY= max([Sy tBoundaries(2,i) tBoundaries(2,i+1)]);
        
        width= maxX- minX;
        heigth= maxY- minY;
        
        %tBoundaryC= [Sx; Sy];
        
        %         for j=1:3:width*heigth
        %             %currentPnt= [ mod(j-1, width)+ minX; floor((j-1)/width)+minY];
        %             %Check if current point is inside the triangle.  Computing the cross
        %             %product but using the primitive cross is much slower, so i
        %             %just compute the last term which is the one i need.
        % %             pntA= currentPnt-tBoundaries(:,i);
        % %             BA= tBoundaries(:,i+1) - tBoundaries(:,i);
        % %             pntB= currentPnt- tBoundaries(:,i+1);
        % %             CB= tBoundaryC- tBoundaries(:,i+1);
        % %             pntC= currentPnt-tBoundaryC;
        % %             AC= tBoundaries(:,i)- tBoundaryC;
        % %
        % %             ab= pntA(1)*BA(2)- pntA(2)*BA(1)
        % %             bc= pntB(1)*CB(2)- pntB(2)*CB(1)
        % %             ca= pntC(1)*AC(2)- pntC(2)*AC(1)
        %
        %             ab= (((mod(j-1, width)+ minX)- tBoundaries(1,i))* (tBoundaries(2,i+1)- tBoundaries(2,i))) - (((floor((j-1)/width)+minY)- tBoundaries(2,i))* (tBoundaries(1,i+1)- tBoundaries(1,i)));
        %             bc= (((mod(j-1, width)+ minX)- tBoundaries(1,i+1))* (Sy- tBoundaries(2,i+1))) - (((floor((j-1)/width)+minY)- tBoundaries(2,i+1))* (Sx- tBoundaries(1,i+1)));
        %             ca= (((mod(j-1, width)+ minX)- Sx)* (tBoundaries(2,i)- Sy)) - (((floor((j-1)/width)+minY)- Sy)* (tBoundaries(1,i)- Sx));
        %             %         ab= cross(currentPnt-tBoundaryA, tBoundaryB - tBoundaryA);
        %             %         bc= cross(currentPnt-tBoundaryB, tBoundaryC- tBoundaryB);
        %             %         ca= cross(currentPnt-tBoundaryC, tBoundaryA- tBoundaryC);
        %
        %             if sign(ab)*sign(bc)>0 && sign(bc)*sign(ca)> 0
        %                 innerPoints= [innerPoints error(round(floor((j-1)/width)+minY), round(mod(j-1, width)+ minX))];
        %             end
        %         end
        x= minX+ (1:3:width)';
        y= minY+ (1:3:heigth);
        
        X= x(:, ones(length(y),1));
        X= X(:);
        
        Y= y(ones(1,length(x)),:);
        Y=Y(:);
        
        currentPnt= [X Y];
        
        A= tBoundaries(:,i)';
        B= tBoundaries(:,i+1)';
        C= [Sx Sy];
        pntA= currentPnt-  A(ones(length(currentPnt),1), :); %repmat(tBoundaries(:,i)', length(currentPnt),1);
        pntB= currentPnt - B(ones(length(currentPnt),1), :);%repmat(tBoundaries(:,i+1)', length(currentPnt),1);
        pntC= currentPnt-  C(ones(length(currentPnt),1), :);%repmat([Sx Sy], length(currentPnt), 1);
        BA= tBoundaries(:,i+1)' - tBoundaries(:,i)';
        CB= [Sx Sy]- tBoundaries(:,i+1)';
        AC= tBoundaries(:,i)'- [Sx Sy];
        
        ab= pntA(:,1).*BA(2)- pntA(:,2).*BA(1);
        bc= pntB(:,1).*CB(2)- pntB(:,2).*CB(1);
        ca= pntC(:,1).*AC(2)- pntC(:,2).*AC(1);
        
        innerPointX= round(X((sign(ab).*sign(bc)>0 - sign(bc).*sign(ca)> 0)==0));
        innerPointY= round(Y((sign(ab).*sign(bc)>0 - sign(bc).*sign(ca)> 0)==0));
        linIndexes= ((innerPointX-1).* size(error,1)) + innerPointY;
        
        innerPoints= error(linIndexes); 
        
        meanError= [meanError sum(innerPoints)/length(innerPoints)];
    else
        meanError= [meanError nan];
    end
    
end
end
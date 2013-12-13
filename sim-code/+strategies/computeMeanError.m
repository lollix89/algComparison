function [meanError]= computeMeanError(Sx, Sy, tBoundaries, error)
%This function computes the mean error for every allowed directions taking
%the mean of the errors falling into each triangle.

%Input:
%       Sx, Sy:             current position of the robot
%       tBoundaries:        coordinates of the boundaries of every triangle
%       error:              error map used to compute the mean in every
%                           triangle

meanError= zeros(1, size(tBoundaries, 2)/2);

for i=1:2:size(tBoundaries, 2)-1
    if all(~isnan(tBoundaries(:,i))) && all(~isnan(tBoundaries(:,i+1)))
        %find square boundaries for every triangle (common vertex is the current position)
        minX= min([Sx tBoundaries(1,i) tBoundaries(1,i+1)]);
        maxX= max([Sx tBoundaries(1,i) tBoundaries(1,i+1)]);
        minY= min([Sy tBoundaries(2,i) tBoundaries(2,i+1)]);
        maxY= max([Sy tBoundaries(2,i) tBoundaries(2,i+1)]);
        
        width= maxX- minX;
        heigth= maxY- minY;
        
        x= minX+ (1:width)';
        y= minY+ (1:heigth);
        X= x(:, ones(length(y),1));
        X= X(:);
        Y= y(ones(1,length(x)),:);
        Y=Y(:);
        currentPnt= [X Y];
        
        A= tBoundaries(:,i)';
        B= tBoundaries(:,i+1)';
        C= [Sx Sy];
        pntA= currentPnt-  A(ones(length(currentPnt),1), :);
        pntB= currentPnt - B(ones(length(currentPnt),1), :);
        pntC= currentPnt-  C(ones(length(currentPnt),1), :);
        BA= tBoundaries(:,i+1)' - tBoundaries(:,i)';
        CB= [Sx Sy]- tBoundaries(:,i+1)';
        AC= tBoundaries(:,i)'- [Sx Sy];
        
        %compute the third term of the vectorial product to find out if the
        %point falls into the current triangle
        ab= pntA(:,1).*BA(2)- pntA(:,2).*BA(1);
        bc= pntB(:,1).*CB(2)- pntB(:,2).*CB(1);
        ca= pntC(:,1).*AC(2)- pntC(:,2).*AC(1);
        innerPointX= round(X((sign(ab).*sign(bc)>0 - sign(bc).*sign(ca)> 0)==0));
        innerPointY= round(Y((sign(ab).*sign(bc)>0 - sign(bc).*sign(ca)> 0)==0));
        linIndexes= ((innerPointX-1).* size(error,1)) + innerPointY;
        innerPoints= error(linIndexes);
        
        meanError(ceil(i/2))= sum(innerPoints)/length(innerPoints);
    else
        meanError(ceil(i/2))=  nan;
    end
end
end
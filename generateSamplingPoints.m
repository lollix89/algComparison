function generateSamplingPoints()

plotOn=0;
trendOrder=0;

%ouvre un fichier ou le créé
config=2;
fid = fopen('./adaptative_sampling/bed.txt','w');


Lx=201;
Ly=201;
X=[];
%stations
X(:,1)=[24 34 14 94 134 74 94 166 186 174];
X(:,2)=[166 94 22 14 86 66 174 174 106 34];


%build grid
h = 2;
x = h/2:h:Lx-1;
y = h/2:h:Ly-1;
ph = 10;
px = 0:ph:Lx-1;
py = 0:ph:Ly-1;

lx=length(x);
ly=length(y);
lpx=length(px);
lpy=length(py);

%sample vector (nan at position non-sampled)
sVec=ones(lx*ly,1)*nan;%sampling vector
kVec=1:lx*ly;
%pos=x;
for i=1:length(X)
    indX = ceil(X(i,1)/h);
    indY = ceil(X(i,2)/h);
    indK = indX+(indY-1)*length(x);
    fprintf(fid,'%f \n',indK);
    sVec(indK) = 1;
end
for iter=1:150
    X=[];
    indK=kVec(~isnan(sVec));
    [indX,indY] = k2indXindY(indK,lx);
    X(:,1)=x(indX);
    X(:,2)=y(indY); 


%==========================================================================
    if config==1
        [interp,krigE]=kriging(X,Y_,stdV,meanV,fittedModel,fittedParam,trendOrder,px,py);
    elseif config==2%======================================================
        for i=1:lpy
            for j=1:lpx
                krigE(i,j)=ComputeShortestDist(X,px(j),py(i));
            end
        end
    end
    %==========================================================================

    %krigE
    krigEvector=zeros(1,lpx*lpy);
    for i=1:lpy
        for j=1:lpx
            k = j+(i-1)*lpx;
            krigEvector(k)=krigE(i,j);
        end
    end
    [~,kMax]=max(krigEvector);
    kMax=kMax(1);
%==========================================================================
    Sy = ceil(kMax/lpx);
    Sx = kMax -(Sy-1)*lpx;

    indX = max(ceil(px(Sx)/h),1);
    indY = max(ceil(py(Sy)/h),1);
    %Y(i) = field(y(indY)+1,x(indX)+1);
    indK = indX+(indY-1)*length(x);
    fprintf(fid,'%f \n',indK);
    sVec(indK) = 1;


    if plotOn==1
        plotmap(px,py,krigE);
        hold on
        plot(X(:,1),X(:,2),'w+','LineWidth',2)
        plot(px(Sx),py(Sy),'k+','LineWidth',2)
    end
end
fclose(fid);



end



function d=ComputeShortestDist(X,x1,x2)
d=sqrt((X(:,1)-x1).^2+(X(:,2)-x2).^2);
d=min(d);

end

function plotmap(x,y,map)
figure
[~, ch]=contourf(x,y,map,30);
set(ch,'edgecolor','none');
set(gca,'FontSize',16)
xlabel('X [m]','FontSize',14)
ylabel('Y [m]','FontSize', 14)
colorbar
axis('equal')
axis([-1 201 -1 201])
end

function [indX,indY] = k2indXindY(k,lx)
    indY = ceil(k/lx);
    indX = k -(indY-1)*lx;
end

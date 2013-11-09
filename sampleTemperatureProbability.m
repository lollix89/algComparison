%sample from the probability distribution on each cell and returns a map
%of temperatures
function sampledTemperatureMap= sampleTemperatureProbability(posterior, temperatureVector, X, Y, samples, delta)

sampledTemperatureMap= nan(size(posterior,1), size(posterior,2));


% for x=1:size(posterior, 1)
%     for y=1:size(posterior, 2)
%
% %         c = cumsum(reshape(posterior(x,y,:), 1, size(posterior,3)));
% %         r = rand(1,1);
% %         e = [0,c];
% %         [~,bin] = histc(r,e);
% %         sampledTemperatureMap(x,y) = temperatureVector(bin);
%           [~,i]= max( reshape(posterior(x,y,:), 1, size(posterior,3)));
%           sampledTemperatureMap(x,y) = temperatureVector(i);
%     end
% end

for i=1:size(X)
    %[~,idx]= max( reshape(posterior(floor(X(i)/delta)+1,floor(Y(i)/delta)+1,:), 1, size(posterior,3)));
    sampledTemperatureMap(floor((X(i)+1)/delta)+1,floor((Y(i)+1)/delta)+1) = samples(i);
end

 sampledTemperatureMap= interpolate(sampledTemperatureMap, 1);
% figure(2)
% [~, ch]=contourf(1:5:201,1:5:201,sampledTemperatureMap,30);
% set(ch,'edgecolor','none');
% set(gca,'FontSize',16)
% axis('equal')
% axis([-2 202 -2 202])
% drawnow

end

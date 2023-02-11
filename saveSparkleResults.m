
for i=1:10
    sparkleResults(i,:) = matfile(['/Users/oliverbroadrick/Desktop/glitter-stuff/feb10/' 'sparkleResults' num2str(i)]).rotAndIntrinsics2;
end
save(['/Users/oliverbroadrick/Desktop/glitter-stuff/feb10/' 'sparkleResults'], "sparkleResults");
function printRow(rowName, rowVals)
    w = '10'; dec = '4';
    s = '%15s ';
    s = [s '(%.3f, %.3f, %.3f)'];
    s = [s '(%.3f, %.3f, %.3f)'];
    for i=1:size(rowVals,2)-6
        s = [s '%' w '.' dec 'f '];
    end
    s = [s '\n'];
    fprintf(s, rowName, rowVals);
end
function printRow(rowName, rowVals)
    w = '10'; dec = '4';
    s = '%15s ';
    for i=1:size(rowVals,2)
        s = [s '%' w '.' dec 'f '];
    end
    s = [s '\n'];
    fprintf(s, rowName, rowVals);
end
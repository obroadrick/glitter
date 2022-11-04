% from addy's light-bar-generating code (written in python)

h = 2160;
w = 3840;
verticalBarLocations = [];
horizontalBarLocations = [];
for i=1:5:w
    %img = zeros(h,w);
    shift = 40;
    if i + 40 > w
        shift = w - i;
    end
    %#shift = i + 40 >= 3840? 3940 - i : 40
    %img(:,i:i+shift) = 255;
    verticalBarLocations(ceil(i/5)) = i + shift / 2;
    %{
    % apply gaussian blur, save the image
    _res = Image.fromarray(_img)
    _res1 = _res.filter(ImageFilter.GaussianBlur(radius=10));
    _res1.save("../Optical Research/Inputs/calib{}.jpg".format(i/5))
    %}
end
for i=1:5:h+1
    %img = zeros(h,w);
    shift = 40;
    if i + 40 > h
        shift = h - i;
    end
    
    %_img[i:i+shift:,] = 255;
    horizontalBarLocations(ceil(i/5)) = i + shift / 2;
    %{
    _res = Image.fromarray(_img)
    _res1 = _res.filter(ImageFilter.GaussianBlur(radius=10));
    _res1.save("../Optical Research/Inputs/calib-h{}.
    %}
end
plot(horizontalBarLocations,'marker','X');
figure;
plot(verticalBarLocations,'marker','X');

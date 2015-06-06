grass = imread('0022.png');
tiles = imread('0008.png');
wood = imread('0029.png');
obstacle = imread('0033.png');
imshow(obstacle);
% sample = grass(110:210, 100:200, :);
% sample = tiles(90:190, 140:240, :);
% sample = wood(4:90, 109:199, :);
sample = obstacle(90:190, 260:320, :);
r = sample(:,:,1); 
g = sample(:,:,2);
b = sample(:,:,3);
m = [mean(r(:)), mean(g(:)), mean(b(:))];

m_n = m / norm(m);
imshow(sample);
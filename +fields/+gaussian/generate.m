% generate field and retrieve data from R
function data = generate(model,width,res,params)

paramstr = num2str(params,'%f ');

% give R a temporary filename to which it can write data
[pn,~,~] = fileparts(mfilename('fullpath'));
fn = tempname;
[status,cmdout] = system(sprintf('Rscript --vanilla %s/generate.R %s %s %d %f %s',pn,fn,model,width,res,paramstr));
assert(status == 0,'R returned error code: %d\n%s',status,cmdout);

% retrieve data written by R
f = load(fn,'-mat','data');
delete(fn);

data = f.data;
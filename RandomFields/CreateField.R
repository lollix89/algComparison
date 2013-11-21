
require(RandomFields)


setwd(getwd()) 
fieldType <-c("SR", "IR", "LR")
fieldQuantity= 100

for(j in 1:length(fieldType)){
	for(i in 1: fieldQuantity){
  		if(fieldType[j] =="SR"){
    		param <- c(25, 25, 0, 10)} # mean, variance, nugget, scale(range)
  		else if(fieldType[j] =="IR"){
    		param <- c(25, 25, 0, 20)
  		}
  		else if(fieldType[j] =="LR"){
    		param <- c(25, 25, 0, 40)
  		}
  
  	model <- "spherical"
  	RFparameters(PracticalRange=FALSE)
  	p <- 1:10
  	field <- GaussRF(x=p, y=p, grid=TRUE, model=model, param=param)
  
  	# another grid, where values are to be simulated
  	step <- 1 # or 0.3
  	x <- seq(0, 299, step)
  	# standardisation of the output
  
  
  	#conditional simulation
  	krige.method <- "O" ## random field assumption corresponding to
  	## those of ordinary kriging
  
  	cz <- CondSimu(krige.method, x, x, grid=TRUE,
                 model=model, param=param,
                 given=expand.grid(p,p),# if data are given on a grid
                 # then expand the grid first
                 data=field)
  
  	nameOutput=paste("RandField_", fieldType[j],"_No",i+ (j-1)* fieldQuantity,".csv", sep = "")
  	write.table(cz,file=nameOutput,sep=",",row.names=F, col.names=F) 
	}
}

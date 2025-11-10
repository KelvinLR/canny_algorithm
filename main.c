#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define PI 3.14159265
#define ALTURA 89
#define LARGURA 89
#define BLOCK_HEIGHT 32
#define GAUSS_MARGIN 2
#define SOBEL_MARGIN 1

uint8_t img[ALTURA][LARGURA]; // input e intermediários
uint8_t output[ALTURA][LARGURA]; // resultado

// --- leitura/salvamento PGM ---
void ler_pgm(const char* filename) {
    FILE* f = fopen(filename, "r");
    if (!f) { printf("Erro ao abrir arquivo\n"); exit(1); }
    char tipo[3]; int largura, altura, maxval;
    fscanf(f, "%2s", tipo); if (tipo[0]!='P'||tipo[1]!='2') { printf("Formato inválido\n"); exit(2);}
    int c; do { c = fgetc(f); if (c=='#') while(fgetc(f)!='\n'); } while(c=='#'); ungetc(c,f);
    fscanf(f,"%d%d",&largura,&altura); fscanf(f,"%d",&maxval);
    if (largura!=LARGURA||altura!=ALTURA) { printf("Dimensões %dx%d erradas\n", largura, altura); exit(3);}
    for (int i=0;i<ALTURA;i++) for(int j=0;j<LARGURA;j++) fscanf(f,"%d", &img[i][j]);
    fclose(f);
}
void salvar_pgm(const char* nome, uint8_t matriz[ALTURA][LARGURA]) {
    FILE* f = fopen(nome, "w");
    fprintf(f, "P2\n%d %d\n255\n", LARGURA, ALTURA);
    for (int i=0;i<ALTURA;i++) { for(int j=0;j<LARGURA;j++) fprintf(f, "%d ", matriz[i][j]); fprintf(f, "\n"); }
    fclose(f);
}

// --- bloco temporário para todas etapas ---
uint8_t block_in[BLOCK_HEIGHT+2*GAUSS_MARGIN][LARGURA];
double block_gauss[BLOCK_HEIGHT][LARGURA];
double block_mag[BLOCK_HEIGHT][LARGURA];
double block_dir[BLOCK_HEIGHT][LARGURA];
uint8_t block_suppress[BLOCK_HEIGHT][LARGURA];
uint8_t block_thresh[BLOCK_HEIGHT][LARGURA];

// --- Gausssiano no bloco ---
void filtro_gaussiano_block(uint8_t in[BLOCK_HEIGHT+2*GAUSS_MARGIN][LARGURA], double out[BLOCK_HEIGHT][LARGURA], int bh) {
    double kernel[5][5] = {
        {2.0/159.0,4.0/159.0,5.0/159.0,4.0/159.0,2.0/159.0},
        {4.0/159.0,9.0/159.0,12.0/159.0,9.0/159.0,4.0/159.0},
        {5.0/159.0,12.0/159.0,15.0/159.0,12.0/159.0,5.0/159.0},
        {4.0/159.0,9.0/159.0,12.0/159.0,9.0/159.0,4.0/159.0},
        {2.0/159.0,4.0/159.0,5.0/159.0,4.0/159.0,2.0/159.0}
    };
    for(int i=GAUSS_MARGIN;i<bh+GAUSS_MARGIN;i++)for(int j=0;j<LARGURA;j++){
        double sum=0;
        for(int m=-GAUSS_MARGIN;m<=GAUSS_MARGIN;m++)
            for(int n=-GAUSS_MARGIN;n<=GAUSS_MARGIN;n++){
                int r=i+m,c=j+n;
                if(r<0)r=0;else if(r>=bh+2*GAUSS_MARGIN)r=bh+2*GAUSS_MARGIN-1;
                if(c<0)c=0;else if(c>=LARGURA)c=LARGURA-1;
                sum+=in[r][c]*kernel[m+GAUSS_MARGIN][n+GAUSS_MARGIN];
            }
        if(sum<0)sum=0; if(sum>255)sum=255;
        out[i-GAUSS_MARGIN][j]=sum;
    }
}

// --- Sobel + gradientes ---
void sobel_block(double in[BLOCK_HEIGHT][LARGURA], double mag[BLOCK_HEIGHT][LARGURA], double dir[BLOCK_HEIGHT][LARGURA], int bh) {
    int gx[3][3] = {{-1,0,1},{-2,0,2},{-1,0,1}}, gy[3][3]={{1,2,1},{0,0,0},{-1,-2,-1}};
    for(int i=1;i<bh-1;i++)for(int j=1;j<LARGURA-1;j++){
        double sx=0,sy=0;
        for(int m=-1;m<=1;m++)for(int n=-1;n<=1;n++){
            sx+=in[i+m][j+n]*gx[m+1][n+1];
            sy+=in[i+m][j+n]*gy[m+1][n+1];
        }
        mag[i][j]=sqrt(sx*sx+sy*sy);
        double angle=atan2(sy,sx)*180.0/PI; if(angle<0)angle+=360.0;
        dir[i][j]=angle;
    }
    for(int i=0;i<bh;i++)for(int j=0;j<LARGURA;j++)
        if(i==0||j==0||i==bh-1||j==LARGURA-1) mag[i][j]=dir[i][j]=0;
}

// --- Normalização, supressão, threshold, histerese no bloco ---
double busca_max_mag(double mag[BLOCK_HEIGHT][LARGURA], int bh) {
    double maxv=0;
    for(int i=0;i<bh;i++)for(int j=0;j<LARGURA;j++)
        if(mag[i][j]>maxv)maxv=mag[i][j];
    return maxv;
}

void non_max_suppression_block(double mag[BLOCK_HEIGHT][LARGURA], double dir[BLOCK_HEIGHT][LARGURA], uint8_t out[BLOCK_HEIGHT][LARGURA], int bh){
    for(int r=1;r<bh-1;r++)for(int c=1;c<LARGURA-1;c++){
        double angle=dir[r][c], g=mag[r][c], bp=0,ap=0;
        int by,bx,ay,ax;
        if((0<=angle&&angle<22.5)||(337.5<=angle&&angle<=360)){ay=by=r;bx=c-1;ax=c+1;}
        else if((22.5<=angle&&angle<67.5)||(202.5<=angle&&angle<247.5)){by=r+1;bx=c-1;ay=r-1;ax=c+1;}
        else if((67.5<=angle&&angle<112.5)||(247.5<=angle&&angle<292.5)){by=r-1;ay=r+1;bx=ax=c;}
        else{by=r-1;bx=c-1;ay=r+1;ax=c+1;}
        bp=mag[by][bx];ap=mag[ay][ax];
        out[r][c]=(g>=bp&&g>=ap)?(uint8_t)g:0;
    }
    for(int j=0;j<LARGURA;j++){out[0][j]=0;out[bh-1][j]=0;}
    for(int i=0;i<bh;i++){out[i][0]=0;out[i][LARGURA-1]=0;}
}

void threshold_block(uint8_t in[BLOCK_HEIGHT][LARGURA], int low, int high, int weak, uint8_t out[BLOCK_HEIGHT][LARGURA], int bh) {
    int strong=255;
    for(int i=0;i<bh;i++)for(int j=0;j<LARGURA;j++)
        out[i][j]=(in[i][j]>=high)?strong:((in[i][j]>=low)?weak:0);
}

int has_strong_neighbor(const uint8_t img[BLOCK_HEIGHT][LARGURA],int l,int c,int bh){
    int strong=255,movs[]={-1,0,1};
    for(int i=0;i<3;i++){int nl=l+movs[i];if(nl<0||nl>=bh)continue;
        for(int j=0;j<3;j++){int nc=c+movs[j];if(nc<0||nc>=LARGURA)continue;
            if(img[nl][nc]==strong)return 1;}}return 0;}

void hysteresis_block(uint8_t img[BLOCK_HEIGHT][LARGURA], int weak, int bh){
    int strong=255;
    for(int i=1;i<bh-1;i++)for(int j=1;j<LARGURA-1;j++)
        if(img[i][j]==weak)
            img[i][j]=has_strong_neighbor(img,i,j,bh)?strong:0;
}

// --- PIPELINE EM BLOCOS ---
void canny_pipeline_blocks(){
    double global_max = 0.0;
    // 1° passo: máximo global
    for(int pos=0;pos<ALTURA;){
        int bh=BLOCK_HEIGHT;if(pos+bh>ALTURA)bh=ALTURA-pos;
        // Copia bloco de entrada com margem
        for(int i=0;i<bh+2*GAUSS_MARGIN;i++){
            int src_i=pos+i-GAUSS_MARGIN;
            if(src_i<0)src_i=0;else if(src_i>=ALTURA)src_i=ALTURA-1;
            for(int j=0;j<LARGURA;j++) block_in[i][j]=img[src_i][j];
        }
        filtro_gaussiano_block(block_in, block_gauss, bh);
        sobel_block(block_gauss, block_mag, block_dir, bh);
        double m = busca_max_mag(block_mag, bh); if(m>global_max)global_max=m;
        pos+=bh;
    }

    if(global_max<=0)global_max=1.0;
    // 2° passo: pipeline completo
    for(int pos=0;pos<ALTURA;){
        int bh=BLOCK_HEIGHT;if(pos+bh>ALTURA)bh=ALTURA-pos;
        for(int i=0;i<bh+2*GAUSS_MARGIN;i++){
            int src_i=pos+i-GAUSS_MARGIN;
            if(src_i<0)src_i=0;else if(src_i>=ALTURA)src_i=ALTURA-1;
            for(int j=0;j<LARGURA;j++) block_in[i][j]=img[src_i][j];
        }
        
        filtro_gaussiano_block(block_in, block_gauss, bh);
        sobel_block(block_gauss, block_mag, block_dir, bh);
        // normaliza magnitude
        for(int i=0;i<bh;i++)for(int j=0;j<LARGURA;j++)
            block_mag[i][j]=block_mag[i][j]*255.0/global_max;
        non_max_suppression_block(block_mag, block_dir, block_suppress, bh);
        threshold_block(block_suppress, 5, 20, 75, block_thresh, bh);
        hysteresis_block(block_thresh, 75, bh);
        for(int i=0;i<bh;i++)for(int j=0;j<LARGURA;j++)
            output[pos+i][j] = block_thresh[i][j];
        pos += bh;
    }
}

int main(int argc, char *argv[]) {
    if(argc < 3) {
        printf("uso: %s <input.pgm> <output.pgm>\n", argv[0]); return 1;
    }
    ler_pgm(argv[1]);
    canny_pipeline_blocks();
    salvar_pgm(argv[2], output);
    printf("pipeline em blocos finalizado\n");
    return 0;
}
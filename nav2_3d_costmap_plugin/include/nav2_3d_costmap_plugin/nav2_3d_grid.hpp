
namespace nav2_3d_grid
{
    class vdbBasedGrid
    {
        public:
            vdbBasedGrid(
                //parameters 
            );
            ~vdbBasedGrid(void);

            // 标记grid
            void Mark();
            // 清除标记信息
            void Clear();
            // 稀疏处理功能
            void Operator();

            bool ResetGrid(void);
            bool SaveGrid();

        protected:
            void initializedGrid(void);
            void Grid2OccupancyGrid();
            // STVL 提供了 worldtoindex/indextoworld 方法完成转换
            
        
            // parameters

    }

}
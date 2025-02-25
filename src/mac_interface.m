#import "Graphics.h"
#import <Foundation/Foundation.h>

#pragma mark - PGEBuffer

// Define an Objective-C class to represent a buffer.
@interface PGEBuffer : NSObject
@property (nonatomic, copy) NSString *name;
@property (nonatomic, assign) int size;
@property (nonatomic, strong) NSMutableData *data;
@end

@implementation PGEBuffer
@end

#pragma mark - PGETexture

// Define an Objective-C class to represent a texture.
@interface PGETexture : NSObject
@property (nonatomic, copy) NSString *name;
@property (nonatomic, assign) int width;
@property (nonatomic, assign) int height;
@property (nonatomic, strong) NSData *data;
@end

@implementation PGETexture
@end

#pragma mark - PGEPipeline

// Define an Objective-C class to represent a pipeline.
@interface PGEPipeline : NSObject
@property (nonatomic, copy) NSString *name;
@end

@implementation PGEPipeline
@end

#pragma mark - C Interface Implementation

Buffer* pge_create_buffer(const char* name, int size) {
    PGEBuffer *buffer = [[PGEBuffer alloc] init];
    buffer.name = [NSString stringWithUTF8String:name];
    buffer.size = size;
    buffer.data = [NSMutableData dataWithLength:size];
    return (Buffer *)buffer;
}

void pge_destroy_buffer(Buffer* buffer) {
    // ARC automatically manages memory.
    // If additional cleanup is required, perform it here.
}

Texture* pge_create_texture(const char* name, void *data, int width, int height) {
    PGETexture *texture = [[PGETexture alloc] init];
    texture.name = [NSString stringWithUTF8String:name];
    texture.width = width;
    texture.height = height;
    if (data) {
        // For example, assume 4 bytes per pixel (e.g., RGBA).
        texture.data = [NSData dataWithBytes:data length:(width * height * 4)];
    }
    return (Texture *)texture;
}

void pge_destroy_texture(Texture* texture) {
    // ARC automatically manages memory.
}

Pipeline* pge_create_pipeline(const char* name) {
    PGEPipeline *pipeline = [[PGEPipeline alloc] init];
    pipeline.name = [NSString stringWithUTF8String:name];
    return (Pipeline *)pipeline;
}

void pge_write_buffer(Buffer* buffer, void* data, int size) {
    PGEBuffer *pgeBuffer = (PGEBuffer *)buffer;
    // Write up to the minimum of 'size' and the buffer's capacity.
    int copySize = MIN(size, pgeBuffer.size);
    [pgeBuffer.data replaceBytesInRange:NSMakeRange(0, copySize) withBytes:data];
}